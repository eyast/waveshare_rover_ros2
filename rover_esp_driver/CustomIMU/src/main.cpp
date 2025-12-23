#include <Arduino.h>
#include "config.h"
#include "i2c_helpers.h"
#include "qmi8658c.h"
#include "ak09918c.h"
#include "madgwick.h"
#include "motioncal_output.h"

// =============================================================================
// QMI8658C + AK09918C IMU Driver - MotionCal Compatible
// =============================================================================
//
// Outputs data in PJRC MotionCal format for magnetometer calibration.
// After calibration, the Madgwick filter provides orientation estimation.
//
// =============================================================================

// Sensor instances
QMI8658C imu(QMI8658C_ADDR);
AK09918C mag(AK09918C_ADDR);
MadgwickFilter filter(MADGWICK_BETA);

// Timing
uint32_t last_output_ms = 0;
uint32_t last_update_us = 0;

// Sensor status
bool imu_ok = false;
bool mag_ok = false;

// =============================================================================
// Serial Command Processing
// =============================================================================

void process_serial_commands() {
    static char cmd_buf[64];
    static uint8_t cmd_idx = 0;

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (cmd_idx > 0) {
                cmd_buf[cmd_idx] = '\0';

                // Process MotionCal calibration commands
                if (strncmp(cmd_buf, "Cal1:", 5) == 0) {
                    // Parse Cal1 data from MotionCal
                    // Format: Cal1:ax,ay,az,mx,my,mz,s00,s01,s02,s10
                    Serial.println("Cal1 received");
                } else if (strncmp(cmd_buf, "Cal2:", 5) == 0) {
                    // Parse Cal2 data from MotionCal
                    Serial.println("Cal2 received");
                } else if (strcmp(cmd_buf, "RECAL") == 0) {
                    // Re-calibrate gyro
                    imu.calibrate_gyro();
                } else if (strcmp(cmd_buf, "RESET") == 0) {
                    // Reset orientation filter
                    filter.reset();
                    Serial.println("Filter reset");
                }

                cmd_idx = 0;
            }
        } else if (cmd_idx < sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_idx++] = c;
        }
    }
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000);

    Serial.println("\n========================================");
    Serial.println("IMU Driver - MotionCal Compatible");
    Serial.println("========================================\n");

    // Initialize I2C
    i2c_init(I2C_SDA, I2C_SCL, I2C_FREQ);

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

    Serial.println("\nStreaming data for MotionCal...\n");
    Serial.println("Commands: RECAL (recalibrate gyro), RESET (reset filter)\n");

    last_update_us = micros();
    last_output_ms = millis();
}

// =============================================================================
// Main Loop
// =============================================================================

void loop() {
    // Process any incoming serial commands
    process_serial_commands();

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

    // Output at fixed rate
    uint32_t now_ms = millis();
    if (now_ms - last_output_ms >= OUTPUT_INTERVAL_MS) {
        last_output_ms = now_ms;

        // Get latest data
        const QMI8658C_Data& imu_data = imu.get_data();
        const AK09918C_Data& mag_data = mag.get_data();

        // Debug: print actual values before scaling
        // Serial.print("DBG: ax="); Serial.print(imu_data.accel[0], 4);
        // Serial.print(" ay="); Serial.print(imu_data.accel[1], 4);
        // Serial.print(" az="); Serial.println(imu_data.accel[2], 4);

        // Send "Raw:" data in MotionCal format
        // MotionCal expects physical units that it will scale internally:
        //   - accel in g
        //   - gyro in dps (degrees per second)
        //   - mag in uT (microtesla)
        motioncal_send_raw(
            imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
            imu_data.gyro_dps[0], imu_data.gyro_dps[1], imu_data.gyro_dps[2],
            mag_data.mag[0], mag_data.mag[1], mag_data.mag[2]
        );

        // Note: motioncal_send_unified() removed - MotionCal doesn't recognize "Uni:" format
        // Uncomment for debugging if needed (will show as "Unknown Message" in MotionCal):
        // motioncal_send_unified(
        //     imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
        //     imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2],
        //     mag_data.mag[0], mag_data.mag[1], mag_data.mag[2]
        // );
    }
}
