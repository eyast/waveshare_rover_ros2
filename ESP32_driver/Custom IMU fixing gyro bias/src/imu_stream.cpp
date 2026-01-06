#include "imu_stream.h"

static uint32_t last_stream_ms = 0;
float temp;

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
        temp = temperatureRead();

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
        doc["y"] = filter.get_yaw();
        doc["p"] = filter.get_pitch();
        doc["r"] = filter.get_roll();
        doc["t"] = temp;
        
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

float getYaw()   { return filter.get_yaw(); }
float getPitch() { return filter.get_pitch(); }
float getRoll()  { return filter.get_roll(); }

void verify_mag_dip(){
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
}