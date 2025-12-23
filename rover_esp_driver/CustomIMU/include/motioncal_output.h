#ifndef MOTIONCAL_OUTPUT_H
#define MOTIONCAL_OUTPUT_H

#include <Arduino.h>

// =============================================================================
// MotionCal Output Functions
// =============================================================================
//
// Outputs sensor data in formats compatible with PJRC MotionCal tool
// https://www.pjrc.com/store/prop_shield.html
//
// IMPORTANT: MotionCal expects specific scaling for "Raw" values:
//   - Accelerometer: 8192 LSB/g (assumes ±4g full scale)
//   - Gyroscope: 16 LSB/dps (assumes ±2000dps full scale)
//   - Magnetometer: 10 LSB/uT
//
// ASCII format: "Raw:ax,ay,az,gx,gy,gz,mx,my,mz\r\n"
// =============================================================================

// MotionCal expected scaling factors
#define MOTIONCAL_ACCEL_SCALE 8192.0f   // LSB per g
#define MOTIONCAL_GYRO_SCALE  16.0f     // LSB per dps
#define MOTIONCAL_MAG_SCALE   10.0f     // LSB per uT

// Send sensor data in MotionCal format
// Inputs should be in physical units:
//   - accel: g (1g = 9.8 m/s²)
//   - gyro: degrees per second
//   - mag: microtesla (uT)
void motioncal_send_raw(float ax, float ay, float az,
                        float gx, float gy, float gz,
                        float mx, float my, float mz);

// Send sensor data from arrays (physical units)
void motioncal_send_raw(const float accel[3],
                        const float gyro[3],
                        const float mag[3]);

// Send unified sensor data (actual physical units for debugging)
// Format: "Uni:ax,ay,az,gx,gy,gz,mx,my,mz\n"
// Units: accel in m/s², gyro in rad/s, mag in uT
void motioncal_send_unified(float ax, float ay, float az,
                            float gx, float gy, float gz,
                            float mx, float my, float mz);

// Send unified data from arrays
void motioncal_send_unified(const float accel[3],
                            const float gyro[3],
                            const float mag[3]);

// Send quaternion in MotionCal binary packet format
void motioncal_send_quaternion(float q0, float q1, float q2, float q3);

// Send quaternion from array
void motioncal_send_quaternion(const float q[4]);

// Send magnetic calibration data packet
void motioncal_send_mag_cal(int16_t id, int16_t x, int16_t y, int16_t z);

#endif // MOTIONCAL_OUTPUT_H
