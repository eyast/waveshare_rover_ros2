#ifndef MADGWICK_H
#define MADGWICK_H

#include <Arduino.h>

// =============================================================================
// Madgwick AHRS Filter
// =============================================================================
//
// An efficient orientation filter for IMUs with accelerometer, gyroscope,
// and magnetometer (9-DOF). Uses gradient descent to fuse sensor data.
//
// Reference: "An efficient orientation filter for inertial and
//            inertial/magnetic sensor arrays" - Sebastian Madgwick
// =============================================================================

class MadgwickFilter {
public:
    MadgwickFilter(float beta = 0.05f);

    // Update with 9-DOF sensor data (gyro in rad/s, accel in g, mag in uT)
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt);

    // Update with 6-DOF sensor data (no magnetometer)
    void update_imu(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float dt);

    // Get quaternion
    void get_quaternion(float* q0, float* q1, float* q2, float* q3) const;
    const float* get_quaternion() const { return q_; }

    // Get Euler angles (in degrees)
    float get_roll() const;
    float get_pitch() const;
    float get_yaw() const;

    // Set filter gain (higher = faster but noisier)
    void set_beta(float beta) { beta_ = beta; }
    float get_beta() const { return beta_; }

    // Reset to initial orientation
    void reset();

private:
    float q_[4];    // Quaternion [w, x, y, z]
    float beta_;    // Filter gain

    // Fast inverse square root (Quake III algorithm)
    static float inv_sqrt(float x);
};

#endif // MADGWICK_H
