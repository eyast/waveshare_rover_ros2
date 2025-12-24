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
#include <INA219_WE.h>
#include <PID_v2.h>
#include "battery_ctrl.h"
#include "oled_ctrl.h"
#include "ugv_config.h"
#include "json_cmd.h"
#include "motors.h"
#include "uart_ctrl.h"
#include "config.h"
#include "i2c_helpers.h"
#include "qmi8658c.h"
#include "ak09918c.h"
#include "madgwick.h"
#include "mag_calibration.h"
#include "hardcoded_calibration.h"

// -----------------------------------------------------------------------------
// IMU Sensor Instances
// -----------------------------------------------------------------------------
QMI8658C imu(QMI8658C_ADDR);
AK09918C mag(AK09918C_ADDR);
MadgwickFilter filter(MADGWICK_BETA);
MagCalibrationReceiver cal_receiver;

// -----------------------------------------------------------------------------
// IMU State
// -----------------------------------------------------------------------------
bool imu_ok = false;
bool mag_ok = false;
uint32_t last_update_us = 0;

// -----------------------------------------------------------------------------
// Loop Statistics (for debugging)
// -----------------------------------------------------------------------------
static unsigned long loop_count = 0;

// -----------------------------------------------------------------------------
// IMU Streaming (must be after IMU instances are defined)
// -----------------------------------------------------------------------------
#include "imu_stream.h"

// =============================================================================
// Setup
// =============================================================================

void setup() {
    // --- Serial ---
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {}
    delay(1000);

    // --- I2C Bus ---
    i2c_init(I2C_SDA, I2C_SCL, I2C_FREQ);

    // --- IMU (QMI8658C) ---
    imu_ok = imu.begin(ACCEL_FS_SEL, GYRO_FS_SEL, ODR_SEL);
    if (!imu_ok) {
        Serial.println("ERROR: QMI8658C init failed!");
    } else {
        Serial.println("QMI8658C OK");
        imu.calibrate_gyro();
    }

    // --- Magnetometer (AK09918C) ---
    mag_ok = mag.begin(AK_MODE_CONT_100HZ);
    if (!mag_ok) {
        Serial.println("ERROR: AK09918C init failed!");
    } else {
        Serial.println("AK09918C OK");
        apply_hardcoded_calibration(mag);
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

    if (InfoPrint == 1) {
        Serial.println("Setup complete.");
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
    loop_count++;
}
