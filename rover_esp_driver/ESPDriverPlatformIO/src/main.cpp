// Devices found
// 0C - Must be the compass, AKAK09918
// 3C - OLED
// 6B - QMI8658_ADDR - https://www.waveshare.com/w/upload/3/36/QMI8658C_datasheet_rev_0.8.pdf
// 6B provides temperature, ax, ay, az, Gx, Gy, Gz
// 42 - INA219: Voltage/Current

#include <ArduinoJson.h>
JsonDocument jsonCmdReceive;
JsonDocument jsonInfoSend;
JsonDocument jsonInfoHttp;


//#include <SCServo.h>
#include <Adafruit_SSD1306.h> // OLED driver
#include <INA219_WE.h>
#include <PID_v2.h>
#include "battery_ctrl.h" // custom functions to initialize and function INA
#include "oled_ctrl.h" // Important, because timekeeping is kept here...
#include "ugv_config.h" // config variables (addresses etc)
#include "json_cmd.h" // List of JSON commands
#include "motors.h"
#include "uart_ctrl.h" // Serial bus

#include "config.h"
#include "i2c_helpers.h"
#include "qmi8658c.h"
#include "ak09918c.h"
#include "madgwick.h"
#include "motioncal_output.h"
#include "mag_calibration.h"
#include "hardcoded_calibration.h"

#define MAIN_LOOP_RATE_HZ     100
#define MAIN_LOOP_PERIOD_MS   (1000 / MAIN_LOOP_RATE_HZ)  // 10ms

// For debugging: track actual loop rate
static unsigned long loop_count = 0;
static unsigned long last_rate_print = 0;
static float actual_loop_rate = 0.0f;

// IMU streaming timing
static uint32_t last_imu_stream_ms = 0;
static float last_dt = 0.01f;

// Sensor instances
QMI8658C imu(QMI8658C_ADDR);
AK09918C mag(AK09918C_ADDR);
MadgwickFilter filter(MADGWICK_BETA);

// Calibration receiver
MagCalibrationReceiver cal_receiver;

// Timing
uint32_t last_output_ms = 0;
uint32_t last_update_us = 0;

// Sensor status
bool imu_ok = false;
bool mag_ok = false;

// Send IMU stream data as JSON when streaming is enabled
void sendIMUStreamData() {
    if (!imu_stream_enabled) return;

    uint32_t now_ms = millis();
    if (now_ms - last_imu_stream_ms < OUTPUT_INTERVAL_MS) return;
    last_imu_stream_ms = now_ms;

    // Get sensor data
    const QMI8658C_Data& imu_data = imu.get_data();
    const AK09918C_Data& mag_data = mag.get_data();

    // Build JSON response
    jsonInfoSend.clear();
    jsonInfoSend["T"] = FEEDBACK_IMU_STREAM;

    // Accelerometer (g)
    jsonInfoSend["ax"] = imu_data.accel[0];
    jsonInfoSend["ay"] = imu_data.accel[1];
    jsonInfoSend["az"] = imu_data.accel[2];

    // Gyroscope (rad/s)
    jsonInfoSend["gx"] = imu_data.gyro[0];
    jsonInfoSend["gy"] = imu_data.gyro[1];
    jsonInfoSend["gz"] = imu_data.gyro[2];

    // Magnetometer (uT, calibrated)
    jsonInfoSend["mx"] = mag_data.mag[0];
    jsonInfoSend["my"] = mag_data.mag[1];
    jsonInfoSend["mz"] = mag_data.mag[2];

    // Orientation (degrees)
    jsonInfoSend["pitch"] = filter.get_pitch();
    jsonInfoSend["roll"] = filter.get_roll();
    jsonInfoSend["yaw"] = filter.get_yaw();

    // Delta time
    jsonInfoSend["dt"] = last_dt;

    serializeJson(jsonInfoSend, Serial);
    Serial.println();
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    i2c_init(I2C_SDA, I2C_SCL, I2C_FREQ);
    //Wire.begin(S_SDA, S_SCL);
    while(!Serial) {}
    delay(1000);

    // Initialize IMU (accelerometer + gyroscope)
    imu_ok = imu.begin(ACCEL_FS_SEL, GYRO_FS_SEL, ODR_SEL);
    if (!imu_ok) {
        Serial.println("ERROR: QMI8658C init failed!");
    } else {
        Serial.println("QMI8658C OK");
    }

    // Initialize magnetometer
    mag_ok = mag.begin(AK_MODE_CONT_100HZ);
    if (!mag_ok) {
        Serial.println("ERROR: AK09918C init failed!");
    } else {
        Serial.println("AK09918C OK");
    }

    // Calibrate gyroscope (device must be stationary)
    if (imu_ok) {
        imu.calibrate_gyro();
    }

    if (mag_ok) {
        apply_hardcoded_calibration(mag);
    }



    ina219_init();
    inaDataUpdate();
    mm_settings(mainType, moduleType);
    init_oled();
    screenLine_0 = "Custom Waveshare";
    screenLine_1 = "Rover Driver";
    screenLine_2 = "";
    screenLine_3 = "Starting...";
    oled_update();
    delay(1000);
    screenLine_2 = screenLine_3;
    screenLine_3 = "Initialize 12V-switch ctrl";
    oled_update();
    if(InfoPrint == 1){Serial.println("Initialize the pins used for 12V-switch ctrl.");}
    motionPinInit();
    pidControllerInit();

}

void loop() {
  static unsigned long last_loop_time = 0;
  unsigned long loop_start = millis();

  serialCtrl();

  // Read sensors
  bool imu_read_ok = imu.read();
  bool mag_read_ok = mag.read();

  // Update orientation filter
  if (imu_read_ok) {
      // Calculate delta time
      uint32_t now_us = micros();
      float dt = (now_us - last_update_us) / 1000000.0f;
      last_update_us = now_us;

      // Sanity check dt
      if (dt <= 0 || dt > 0.1f) {
          dt = 0.01f;
      }
      last_dt = dt;

      // Get sensor data
      const QMI8658C_Data& imu_data = imu.get_data();
      const AK09918C_Data& mag_data = mag.get_data();

      // Update Madgwick filter
      if (mag_read_ok) {
          filter.update(
              imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2],
              imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
              mag_data.mag[0], mag_data.mag[1], mag_data.mag[2],
              dt
          );
      } else {
          // Use IMU-only update when magnetometer data not available
          filter.update_imu(
              imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2],
              imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
              dt
          );
      }
  }

  // Send IMU stream data if enabled
  sendIMUStreamData();

  oledInfoUpdate();
  heartBeatCtrl();
  loop_count++;
}