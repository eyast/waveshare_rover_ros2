/*
 * Madgwick AHRS Filter
 * 
 * An efficient orientation filter for IMUs with accelerometer, gyroscope,
 * and magnetometer (9-DOF). Uses gradient descent to fuse sensor data.
 *
 * Features:
 *   - Sensor-based initialization (instant correct orientation)
 *   - Adaptive beta (motion-aware gain adjustment)
 *   - Fast convergence mode for initial alignment
 *
 * Reference: "An efficient orientation filter for inertial and
 *            inertial/magnetic sensor arrays" - Sebastian Madgwick
 */

#ifndef MADGWICK_H
#define MADGWICK_H

#include <Arduino.h>
#include "config.h"

class MadgwickFilter {
public:
    MadgwickFilter(float beta = FILTER_BETA_DEFAULT);
    
    // =========================================================================
    // Initialization
    // =========================================================================
    
    // Initialize from accelerometer only (6-DOF) - sets roll/pitch, yaw=0
    void init_from_accel(float ax, float ay, float az);
    
    // Initialize from full 9-DOF sensors - instant correct orientation
    void init_from_sensors(float ax, float ay, float az,
                           float mx, float my, float mz);
    
    // Reset to identity quaternion [1,0,0,0]
    void reset();
    
    // =========================================================================
    // Update Methods
    // =========================================================================
    
    // Update with 9-DOF sensor data
    // gx,gy,gz: gyroscope in rad/s
    // ax,ay,az: accelerometer in g
    // mx,my,mz: magnetometer in µT
    // dt: time step in seconds
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt);
    
    // Update with 6-DOF sensor data (no magnetometer)
    void update_imu(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float dt);
    
    // =========================================================================
    // Output Methods
    // =========================================================================
    
    // Get quaternion [w, x, y, z]
    void get_quaternion(float* q0, float* q1, float* q2, float* q3) const;
    const float* quaternion() const { return q_; }
    
    // Get Euler angles in degrees
    float roll() const;
    float pitch() const;
    float yaw() const;
    
    // =========================================================================
    // Configuration
    // =========================================================================
    
    // Set base filter gain (beta)
    void set_beta(float beta) { base_beta_ = beta; }
    float get_beta() const { return base_beta_; }
    
    // Adaptive beta (motion-based adjustment)
    void set_adaptive(bool enabled) { adaptive_enabled_ = enabled; }
    bool is_adaptive() const { return adaptive_enabled_; }
    void set_beta_range(float stationary, float motion);
    void set_motion_threshold(float threshold) { motion_threshold_ = threshold; }
    
    // Fast convergence mode
    void start_fast_convergence(uint32_t duration_ms = FILTER_FAST_CONV_DURATION,
                                 float beta = FILTER_FAST_CONV_BETA);
    void end_fast_convergence();
    bool in_fast_convergence() const { return fast_active_; }
    
    // =========================================================================
    // Diagnostics
    // =========================================================================
    
    // Convergence rate (low = stable, high = still converging)
    float convergence_rate() const { return conv_rate_; }
    bool is_converged() const { return conv_rate_ < 0.01f && !fast_active_; }
    float active_beta() const { return beta_; }

private:
    float q_[4];            // Quaternion [w, x, y, z]
    float q_prev_[4];       // Previous quaternion for convergence tracking
    float beta_;            // Current active beta
    float base_beta_;       // Base beta for normal operation
    
    // Adaptive beta
    bool adaptive_enabled_;
    float stat_beta_;       // Beta when stationary
    float motion_beta_;     // Beta when moving
    float motion_threshold_;
    
    // Fast convergence
    bool fast_active_;
    uint32_t fast_start_;
    uint32_t fast_duration_;
    float fast_beta_;
    
    // Convergence tracking
    float conv_rate_;
    
    // Internal methods
    float compute_beta(float gyro_mag);
    void update_convergence();
    static float inv_sqrt(float x);
    static void euler_to_quat(float r, float p, float y, float* q);
};

// Global filter instance
extern MadgwickFilter filter;

#endif // MADGWICK_H
