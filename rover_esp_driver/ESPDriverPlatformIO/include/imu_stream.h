// =============================================================================
// IMU Processing - Filter updates and JSON streaming
// =============================================================================

#ifndef IMU_STREAM_H
#define IMU_STREAM_H

#include <Arduino.h>

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------

static uint32_t last_imu_stream_ms = 0;
static float last_dt = 0.01f;

// -----------------------------------------------------------------------------
// Filter Update
// -----------------------------------------------------------------------------

void updateIMUFilter() {
    bool imu_read_ok = imu.read();
    bool mag_read_ok = mag.read();

    if (!imu_read_ok) return;

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
        filter.update_imu(
            imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2],
            imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
            dt
        );
    }
}

// -----------------------------------------------------------------------------
// JSON Streaming
// -----------------------------------------------------------------------------

void sendIMUStreamData() {
    if (!imu_stream_enabled) return;

    uint32_t now_ms = millis();
    if (now_ms - last_imu_stream_ms < OUTPUT_INTERVAL_MS) return;
    last_imu_stream_ms = now_ms;

    const QMI8658C_Data& imu_data = imu.get_data();
    const AK09918C_Data& mag_data = mag.get_data();

    if (stream_as_json){
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
    else {
        float mag_raw_ut[3] = {
            mag_data.mag_raw[0] * 0.15f,
            mag_data.mag_raw[1] * 0.15f,
            mag_data.mag_raw[2] * 0.15f
        };

        // Uncalibratd data
        motioncal_send_raw(
            imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
            imu_data.gyro_dps[0], imu_data.gyro_dps[1], imu_data.gyro_dps[2],
            mag_raw_ut[0], mag_raw_ut[1], mag_raw_ut[2]
        );
        if (stream_orientiation){
                motioncal_send_orientation(
                    filter.get_yaw(),
                    filter.get_pitch(),
                    filter.get_roll()
        );
        }
    }
}

#endif // IMU_STREAM_H
