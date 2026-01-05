
// -----------------------------------------------------------------------------
// Waveshare Rover custom ESP driver
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <ArduinoJson.h>
#include "websockets.h"
#include <SCServo.h>
#include "config.h"
//#include <INA219_WE.h>
#include "battery_ctrl.h"
#include "oled_ctrl.h"
#include "motors.h"
#include "imu_stream.h"
#include "uart_ctrl.h"
#include "i2c_helpers.h"
#include "hardcoded_calibration.h"
#include "motioncal_output.h"

void setup() {
    //init_oled();
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {}
    delay(1000);

    i2c_init(I2C_SDA, I2C_SCL, I2C_FREQ);
    
    imu_ok = imu.begin(ACCEL_FS_SEL, GYRO_FS_SEL, ODR_SEL);
    if (!imu_ok) {
        Serial.println("ERROR: QMI8658C init failed!");
    } else {
        Serial.println("QMI8658C OK");
    }

    mag_ok = mag.begin(AK_MODE_CONT_100HZ);
    if (!mag_ok) {
        Serial.println("ERROR: AK09918C init failed!");
    } else {
        Serial.println("AK09918C OK");
        apply_hardcoded_calibration(mag);
    }

    
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
        #endif

    }

    // --- Power Monitor (INA219) ---
    ina219_init();
    inaDataUpdate();

    // --- Motor Configuration ---
    mm_settings(mainType, moduleType);
    motionPinInit();
    pidControllerInit();

    // Initialize timing
    last_update_us = micros();
    lastCmdRecvTime = millis(); 
}

void loop() {
    serialCtrl();
    // webSocket.loop();
    updateIMUFilter();
    sendIMUStreamData();
    //heartBeatCtrl();
    //inaDataUpdate();
}
