#ifndef IMU_STREAM_H
#define IMU_STREAM_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"
#include "qmi8658c.h"
#include "ak09918c.h"
#include "madgwick.h"
#include "motioncal_output.h"

// =============================================================================
// IMU Streaming Configuration
// =============================================================================

// Output interval (matches OUTPUT_RATE_HZ in config.h)
#ifndef OUTPUT_INTERVAL_MS
#define OUTPUT_INTERVAL_MS 20  // 50 Hz
#endif

// Enable debug output (prints what goes into Madgwick)
#define IMU_DEBUG_FILTER_INPUT false

// =============================================================================
// External references (defined in main.cpp)
// =============================================================================
extern QMI8658C imu;
extern AK09918C mag;
extern MadgwickFilter filter;
extern bool imu_ok;
extern bool mag_ok;
extern uint32_t last_update_us;

// Stream control (defined in uart_ctrl.h)
extern bool imu_stream_enabled;
extern bool stream_as_json;

// =============================================================================
// Timing state
// =============================================================================
static uint32_t last_stream_ms = 0;

#if IMU_DEBUG_FILTER_INPUT
static uint32_t debug_counter = 0;
#endif

// =============================================================================
// Update IMU Filter
// =============================================================================
void updateIMUFilter() {
    if (!imu_ok) return;
    
    bool imu_read = imu.read();
    bool mag_read = mag_ok ? mag.read() : false;

    if (imu_read) {
        // Calculate dt
        uint32_t now_us = micros();
        float dt = (now_us - last_update_us) / 1000000.0f;
        last_update_us = now_us;

        // Sanity check dt
        if (dt <= 0 || dt > 0.1f) {
            dt = 0.02f;  // Default to 50Hz
        }

        const QMI8658C_Data& imu_data = imu.get_data();
        const AK09918C_Data& mag_data = mag.get_data();

#if IMU_DEBUG_FILTER_INPUT
        if (++debug_counter % 250 == 0) {
            Serial.print("DBG: g=(");
            Serial.print(imu_data.gyro[0], 3); Serial.print(",");
            Serial.print(imu_data.gyro[1], 3); Serial.print(",");
            Serial.print(imu_data.gyro[2], 3); Serial.print(") a=(");
            Serial.print(imu_data.accel[0], 3); Serial.print(",");
            Serial.print(imu_data.accel[1], 3); Serial.print(",");
            Serial.print(imu_data.accel[2], 3); Serial.print(") m=(");
            Serial.print(mag_data.mag[0], 1); Serial.print(",");
            Serial.print(mag_data.mag[1], 1); Serial.print(",");
            Serial.print(mag_data.mag[2], 1); Serial.println(")");
        }
#endif

        // Update filter
        if (mag_read) {
            filter.update(
                imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2],
                imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
                mag_data.mag[0], mag_data.mag[1], mag_data.mag[2],
                dt
            );
        } else {
            filter.update_imu(
                imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2],
                imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
                dt
            );
        }
    }
}

// =============================================================================
// Send IMU Stream Data (for MotionCal or JSON format)
// =============================================================================
void sendIMUStreamData() {
    // Check if streaming is enabled
    if (!imu_stream_enabled) return;
    
    uint32_t now_ms = millis();
    if (now_ms - last_stream_ms < OUTPUT_INTERVAL_MS) {
        return;
    }
    last_stream_ms = now_ms;

    if (!imu_ok) return;

    const QMI8658C_Data& imu_data = imu.get_data();
    const AK09918C_Data& mag_data = mag.get_data();

    if (stream_as_json) {
        // JSON format for control library
        // Using a static JSON document to avoid repeated allocation
        StaticJsonDocument<384> doc;
        doc["T"] = "imu";
        doc["yaw"] = filter.get_yaw();
        doc["pitch"] = filter.get_pitch();
        doc["roll"] = filter.get_roll();
        
        JsonArray a = doc.createNestedArray("a");
        a.add(imu_data.accel[0]);
        a.add(imu_data.accel[1]);
        a.add(imu_data.accel[2]);
        
        JsonArray g = doc.createNestedArray("g");
        g.add(imu_data.gyro_dps[0]);
        g.add(imu_data.gyro_dps[1]);
        g.add(imu_data.gyro_dps[2]);
        
        JsonArray m = doc.createNestedArray("m");
        m.add(mag_data.mag[0]);
        m.add(mag_data.mag[1]);
        m.add(mag_data.mag[2]);
        
        serializeJson(doc, Serial);
        Serial.println();
    } else {
        // MotionCal format (Raw:) - sends RAW magnetometer data for calibration
        float mag_raw_ut[3] = {
            mag_data.mag_raw[0] * 0.15f,
            mag_data.mag_raw[1] * 0.15f,
            mag_data.mag_raw[2] * 0.15f
        };

        motioncal_send_raw(
            imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
            imu_data.gyro_dps[0], imu_data.gyro_dps[1], imu_data.gyro_dps[2],
            mag_raw_ut[0], mag_raw_ut[1], mag_raw_ut[2]
        );
        
        // Also send orientation in MotionCal format
        motioncal_send_orientation(filter.get_yaw(), filter.get_pitch(), filter.get_roll());
    }
}

// =============================================================================
// Get current orientation (for display or other uses)
// These are NOT inline so they can be called from uart_ctrl.h
// =============================================================================
float getYaw()   { return filter.get_yaw(); }
float getPitch() { return filter.get_pitch(); }
float getRoll()  { return filter.get_roll(); }

// =============================================================================
// Print current sensor status (for debugging)
// NOT inline so it can be called from uart_ctrl.h
// =============================================================================
void printIMUDebug() {
    const QMI8658C_Data& imu_data = imu.get_data();
    const AK09918C_Data& mag_data = mag.get_data();

    Serial.println("\n=== IMU Debug ===");
    
    Serial.print("Accel (g): ");
    Serial.print(imu_data.accel[0], 4); Serial.print(", ");
    Serial.print(imu_data.accel[1], 4); Serial.print(", ");
    Serial.println(imu_data.accel[2], 4);
    
    Serial.print("Gyro (dps): ");
    Serial.print(imu_data.gyro_dps[0], 4); Serial.print(", ");
    Serial.print(imu_data.gyro_dps[1], 4); Serial.print(", ");
    Serial.println(imu_data.gyro_dps[2], 4);
    
    Serial.print("Mag (uT): ");
    Serial.print(mag_data.mag[0], 2); Serial.print(", ");
    Serial.print(mag_data.mag[1], 2); Serial.print(", ");
    Serial.println(mag_data.mag[2], 2);

    // Calculate and show dip angle
    float mag_h = sqrtf(mag_data.mag[0]*mag_data.mag[0] + mag_data.mag[1]*mag_data.mag[1]);
    float mag_t = sqrtf(mag_h*mag_h + mag_data.mag[2]*mag_data.mag[2]);
    float dip = atan2f(mag_data.mag[2], mag_h) * 180.0f / PI;
    
    Serial.print("Mag: horiz="); Serial.print(mag_h, 1);
    Serial.print(" total="); Serial.print(mag_t, 1);
    Serial.print(" dip="); Serial.print(dip, 1); Serial.println("°");
    Serial.println("(Melbourne should have dip ≈ -65°)");

    Serial.print("Orientation: Y="); Serial.print(filter.get_yaw(), 1);
    Serial.print(" P="); Serial.print(filter.get_pitch(), 1);
    Serial.print(" R="); Serial.println(filter.get_roll(), 1);
}

#endif // IMU_STREAM_H
