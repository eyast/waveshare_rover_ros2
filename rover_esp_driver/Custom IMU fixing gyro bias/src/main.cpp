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
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "websockets.h"
#include <SCServo.h>
#include <Adafruit_SSD1306.h>
#include "qmi8658c.h"
#include "ak09918c.h"
#include "madgwick.h"
#include "config.h"
QMI8658C imu(QMI8658C_ADDR);
AK09918C mag(AK09918C_ADDR);
MadgwickFilter filter(MADGWICK_BETA);

#include <INA219_WE.h>
#include <PID_v2.h>
#include "battery_ctrl.h"
#include "oled_ctrl.h"
#include "motors.h"
#include "imu_stream.h"
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
uint32_t last_update_ina = 0;

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
        screenLine_2 = "and leveled";
        screenLine_3 = "...";
        oled_update();
        delay(1000);
        
        imu.calibrate_gyro(1000);
        imu.calibrate_accel(1000);
        screenLine_0 = "";
        screenLine_1 = "";
        screenLine_2 = "";
        screenLine_3 = "done";
        oled_update();
        delay(500);
    }

    // --- Verify magnetometer dip angle ---
    if (mag_ok && imu_ok) {
        // Take a few readings to stabilize
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

    // =========================================================================
    // FILTER INITIALIZATION - KEY IMPROVEMENT!
    // =========================================================================
    // Instead of starting at identity [1,0,0,0] and waiting for slow
    // convergence, we initialize from sensor readings for instant alignment.
    
    if (imu_ok) {
        // Get fresh readings
        imu.read();
        const QMI8658C_Data& imu_data = imu.get_data();
        
        if (mag_ok) {
            mag.read();
            const AK09918C_Data& mag_data = mag.get_data();
            
            // Initialize with full 9-DOF - instant correct orientation!
            filter.initialize_from_sensors(
                imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
                mag_data.mag[0], mag_data.mag[1], mag_data.mag[2]
            );
            Serial.println("Filter initialized from accel+mag (9-DOF)");
        } else {
            // Fall back to accel-only (yaw will be 0)
            filter.initialize_from_accel(
                imu_data.accel[0], imu_data.accel[1], imu_data.accel[2]
            );
            Serial.println("Filter initialized from accel only (6-DOF, yaw=0)");
        }
        
        // Configure adaptive beta for motion-aware operation
        #if ADAPTIVE_BETA_ENABLED
        filter.set_adaptive_beta(true);
        filter.set_beta_range(ADAPTIVE_BETA_STATIONARY, ADAPTIVE_BETA_MOTION);
        filter.set_motion_threshold(MOTION_THRESHOLD_RADS);
        Serial.println("Adaptive beta enabled:");
        Serial.print("  Stationary beta: "); Serial.println(ADAPTIVE_BETA_STATIONARY);
        Serial.print("  Motion beta: "); Serial.println(ADAPTIVE_BETA_MOTION);
        Serial.print("  Threshold: "); Serial.print(MOTION_THRESHOLD_RADS);
        Serial.println(" rad/s");
        #endif
        
        // Optional: Start with fast convergence for extra refinement
        // filter.start_fast_convergence(FAST_CONVERGENCE_DURATION_MS, FAST_CONVERGENCE_BETA);
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
    screenLine_0 = "Waveshare Rover";
    screenLine_1 = "Custom Driver";
    screenLine_2 = "v0.96";
    screenLine_3 = "Starting...";
    oled_update();
    delay(500);

    // Show initial orientation
    screenLine_2 = "Y:" + String((int)filter.get_yaw()) + 
                   " P:" + String((int)filter.get_pitch()) + 
                   " R:" + String((int)filter.get_roll());
    screenLine_3 = "Ready";
    oled_update();

    // Initialize timing
    last_update_us = micros();

    if (InfoPrint == 1) {
        Serial.println("\nSetup complete.");
        Serial.println("Filter initialized from sensors - no convergence delay!");
        Serial.println("\nNew commands:");
        Serial.println("  {\"T\":351}                     - Re-init from sensors");
        Serial.println("  {\"T\":352,\"duration\":2000}   - Start fast convergence");
        Serial.println("  {\"T\":353,\"enabled\":1}       - Enable adaptive beta");
        Serial.println("  {\"T\":354}                     - Get convergence status");
        Serial.println("  {\"T\":345}                     - Debug info");
        Serial.println("");
    }
}

// =============================================================================
// Main Loop
// =============================================================================
void loop() {
    lastCmdRecvTime = millis(); 
    serialCtrl();
    webSocket.loop();
    updateIMUFilter();
    sendIMUStreamData();
    oledInfoUpdate();
    heartBeatCtrl();
    inaDataUpdate();
}
