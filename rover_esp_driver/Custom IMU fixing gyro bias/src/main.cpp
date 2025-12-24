// =============================================================================
// Waveshare Rover ESP32 Driver
// =============================================================================
//
// I2C Devices:
//   0x0C - AK09918C Magnetometer
//   0x3C - SSD1306 OLED Display
//   0x6B - QMI8658C IMU (Accel + Gyro)
//   0x42 - INA219 Voltage/Current Sensor
//
// =============================================================================

// -----------------------------------------------------------------------------
// Libraries
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <ArduinoJson.h>
JsonDocument jsonCmdReceive;
JsonDocument jsonInfoSend;
JsonDocument jsonInfoHttp;

#include <Adafruit_SSD1306.h>
#include "qmi8658c.h"
#include "ak09918c.h"
#include "madgwick.h"
#include "mag_calibration.h"
#include "config.h"
QMI8658C imu(QMI8658C_ADDR);
AK09918C mag(AK09918C_ADDR);
MadgwickFilter filter(MADGWICK_BETA);
MagCalibrationReceiver cal_receiver;

#include <INA219_WE.h>
#include <PID_v2.h>
#include "battery_ctrl.h"
#include "oled_ctrl.h"
#include "ugv_config.h"
#include "json_cmd.h"
#include "motors.h"

#include "uart_ctrl.h"
#include "i2c_helpers.h"

#include "hardcoded_calibration.h"
#include "motioncal_output.h"


// -----------------------------------------------------------------------------
// IMU State
// -----------------------------------------------------------------------------
bool imu_ok = false;
bool mag_ok = false;
uint32_t last_update_us = 0;

// -----------------------------------------------------------------------------
// IMU Streaming
// -----------------------------------------------------------------------------
#include "imu_stream.h"

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
    // --- Serial ---
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {}
    delay(1000);

    // --- I2C Bus ---
    i2c_init(I2C_SDA, I2C_SCL, I2C_FREQ);

    // --- OLED-------
    init_oled();

    // --- IMU (QMI8658C) ---
    imu_ok = imu.begin(ACCEL_FS_SEL, GYRO_FS_SEL, ODR_SEL);
    if (!imu_ok) {
        Serial.println("ERROR: QMI8658C init failed!");
    } else {
        Serial.println("QMI8658C OK");
    }

    // --- Magnetometer (AK09918C) ---
    mag_ok = mag.begin(AK_MODE_CONT_100HZ);
    if (!mag_ok) {
        Serial.println("ERROR: AK09918C init failed!");
    } else {
        Serial.println("AK09918C OK");
        apply_hardcoded_calibration(mag);
    }

    // --- IMU Calibration ---
    // Device MUST be stationary and level during this phase!
    if (imu_ok) {
            
        
        screenLine_0 = "Calibrating";
        screenLine_1 = "Please keep steady";
        screenLine_2 = "and leveld";
        screenLine_3 = "...";
        oled_update();
        delay(1000);
        
        imu.calibrate_gyro(1000);
        imu.calibrate_accel(1000);  // NEW: Accelerometer bias calibration
        screenLine_0 = "";
        screenLine_1 = "";
        screenLine_2 = "";
        screenLine_3 = "done";
        oled_update();
        delay(1000);
    }

    // --- Verify magnetometer dip angle ---
    if (mag_ok && imu_ok) {
        // Take a few readings to verify calibration
        for (int i = 0; i < 20; i++) {
            mag.read();
            delay(10);
        }
        
        const AK09918C_Data& mag_data = mag.get_data();
        float mag_h = sqrtf(mag_data.mag[0]*mag_data.mag[0] + mag_data.mag[1]*mag_data.mag[1]);
        float dip = atan2f(mag_data.mag[2], mag_h) * 180.0f / PI;
        
        Serial.print("\nMag dip angle check: ");
        Serial.print(dip, 1);
        Serial.println("°");
        
        if (dip > 0) {
            Serial.println("WARNING: Positive dip in southern hemisphere!");
            Serial.println("         Check MAG_AXIS_Z_SIGN in hardcoded_calibration.h");
        } else if (dip > -50 || dip < -80) {
            Serial.println("WARNING: Dip angle unexpected for Melbourne (~-65°)");
        } else {
            Serial.println("Dip angle looks correct for Melbourne.");
        }
    }

    // --- Power Monitor (INA219) ---
    ina219_init();
    inaDataUpdate();

    // --- Motor Configuration ---
    mm_settings(mainType, moduleType);
    motionPinInit();
    pidControllerInit();

    // --- OLED Display ---
    init_oled();
    screenLine_0 = "Custom Waveshare";
    screenLine_1 = "Rover Driver";
    screenLine_2 = "";
    screenLine_3 = "Starting...";
    oled_update();
    delay(1000);

    screenLine_2 = screenLine_3;
    screenLine_3 = "Ready";
    oled_update();

    // Initialize timing
    last_update_us = micros();

    if (InfoPrint == 1) {
        Serial.println("\nSetup complete.");
        Serial.println("Use JSON commands - see config.h for command types");
        Serial.println("Example: {\"T\":112} to recalibrate all\n");
    }
}

// =============================================================================
// Main Loop
// =============================================================================
void loop() {
    serialCtrl();
    updateIMUFilter();
    sendIMUStreamData();
    oledInfoUpdate();
    heartBeatCtrl();
}
