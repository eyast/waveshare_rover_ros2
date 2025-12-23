#include "motioncal_output.h"

// Helper: Send a byte with 0x7D escape encoding if needed
static void send_encoded_byte(uint8_t b) {
    if (b == 0x7E) {
        Serial.write(0x7D);
        Serial.write(0x5E);
    } else if (b == 0x7D) {
        Serial.write(0x7D);
        Serial.write(0x5D);
    } else {
        Serial.write(b);
    }
}

void motioncal_send_raw(float ax, float ay, float az,
                        float gx, float gy, float gz,
                        float mx, float my, float mz) {
    // Convert physical units to MotionCal's expected "raw" format
    // MotionCal expects:
    //   - Accel: 8192 LSB/g
    //   - Gyro: 16 LSB/dps
    //   - Mag: 10 LSB/uT

    int accel_x = (int)(ax * MOTIONCAL_ACCEL_SCALE);
    int accel_y = (int)(ay * MOTIONCAL_ACCEL_SCALE);
    int accel_z = (int)(az * MOTIONCAL_ACCEL_SCALE);

    int gyro_x = (int)(gx * MOTIONCAL_GYRO_SCALE);
    int gyro_y = (int)(gy * MOTIONCAL_GYRO_SCALE);
    int gyro_z = (int)(gz * MOTIONCAL_GYRO_SCALE);

    int mag_x = (int)(mx * MOTIONCAL_MAG_SCALE);
    int mag_y = (int)(my * MOTIONCAL_MAG_SCALE);
    int mag_z = (int)(mz * MOTIONCAL_MAG_SCALE);

    // Format: "Raw:ax,ay,az,gx,gy,gz,mx,my,mz\n"
    Serial.print("Raw:");
    Serial.print(accel_x); Serial.print(",");
    Serial.print(accel_y); Serial.print(",");
    Serial.print(accel_z); Serial.print(",");
    Serial.print(gyro_x); Serial.print(",");
    Serial.print(gyro_y); Serial.print(",");
    Serial.print(gyro_z); Serial.print(",");
    Serial.print(mag_x); Serial.print(",");
    Serial.print(mag_y); Serial.print(",");
    Serial.println(mag_z);
}

void motioncal_send_raw(const float accel[3],
                        const float gyro[3],
                        const float mag[3]) {
    motioncal_send_raw(accel[0], accel[1], accel[2],
                       gyro[0], gyro[1], gyro[2],
                       mag[0], mag[1], mag[2]);
}

void motioncal_send_unified(float ax, float ay, float az,
                            float gx, float gy, float gz,
                            float mx, float my, float mz) {
    // Output actual physical units (for debugging/verification)
    // Accel: convert from g to m/s² (multiply by 9.8)
    // Gyro: already in rad/s
    // Mag: already in uT
    Serial.print("Uni:");
    Serial.print(ax * 9.80665f); Serial.print(",");
    Serial.print(ay * 9.80665f); Serial.print(",");
    Serial.print(az * 9.80665f); Serial.print(",");
    Serial.print(gx, 4); Serial.print(",");
    Serial.print(gy, 4); Serial.print(",");
    Serial.print(gz, 4); Serial.print(",");
    Serial.print(mx); Serial.print(",");
    Serial.print(my); Serial.print(",");
    Serial.print(mz);
    Serial.println("");
}

void motioncal_send_unified(const float accel[3],
                            const float gyro[3],
                            const float mag[3]) {
    motioncal_send_unified(accel[0], accel[1], accel[2],
                           gyro[0], gyro[1], gyro[2],
                           mag[0], mag[1], mag[2]);
}

void motioncal_send_quaternion(float q0, float q1, float q2, float q3) {
    uint8_t packet[34];
    memset(packet, 0, sizeof(packet));

    // Packet type
    packet[0] = 0x01;

    // Convert quaternion to int16 scaled by 30000
    int16_t q0_i = (int16_t)(q0 * 30000.0f);
    int16_t q1_i = (int16_t)(q1 * 30000.0f);
    int16_t q2_i = (int16_t)(q2 * 30000.0f);
    int16_t q3_i = (int16_t)(q3 * 30000.0f);

    // Pack quaternion at offset 24 (little-endian)
    packet[24] = q0_i & 0xFF;
    packet[25] = (q0_i >> 8) & 0xFF;
    packet[26] = q1_i & 0xFF;
    packet[27] = (q1_i >> 8) & 0xFF;
    packet[28] = q2_i & 0xFF;
    packet[29] = (q2_i >> 8) & 0xFF;
    packet[30] = q3_i & 0xFF;
    packet[31] = (q3_i >> 8) & 0xFF;

    // Send with framing
    Serial.write(0x7E);  // Start frame
    for (int i = 0; i < 34; i++) {
        send_encoded_byte(packet[i]);
    }
    Serial.write(0x7E);  // End frame
}

void motioncal_send_quaternion(const float q[4]) {
    motioncal_send_quaternion(q[0], q[1], q[2], q[3]);
}

void motioncal_send_mag_cal(int16_t id, int16_t x, int16_t y, int16_t z) {
    uint8_t packet[14];
    memset(packet, 0, sizeof(packet));

    // Packet type
    packet[0] = 0x06;

    // ID at offset 6
    packet[6] = id & 0xFF;
    packet[7] = (id >> 8) & 0xFF;

    // XYZ at offset 8
    packet[8] = x & 0xFF;
    packet[9] = (x >> 8) & 0xFF;
    packet[10] = y & 0xFF;
    packet[11] = (y >> 8) & 0xFF;
    packet[12] = z & 0xFF;
    packet[13] = (z >> 8) & 0xFF;

    // Send with framing
    Serial.write(0x7E);
    for (int i = 0; i < 14; i++) {
        send_encoded_byte(packet[i]);
    }
    Serial.write(0x7E);
}
