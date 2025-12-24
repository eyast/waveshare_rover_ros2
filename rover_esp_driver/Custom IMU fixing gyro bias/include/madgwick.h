#ifndef MADGWICK_H
#define MADGWICK_H

#include <Arduino.h>

// =============================================================================
// Madgwick AHRS Filter (Enhanced)
// =============================================================================
//
// An efficient orientation filter for IMUs with accelerometer, gyroscope,
// and magnetometer (9-DOF). Uses gradient descent to fuse sensor data.
//
// ENHANCEMENTS:
//   1. Sensor-based initialization (instant correct orientation)
//   2. Adaptive beta scheduling (fast initial convergence)
//   3. Motion-adaptive beta (trust gyro during motion, accel when still)
//
// Reference: "An efficient orientation filter for inertial and
//            inertial/magnetic sensor arrays" - Sebastian Madgwick
// =============================================================================

class MadgwickFilter {
public:
    MadgwickFilter(float beta = 0.05f);

    // =========================================================================
    // INITIALIZATION METHODS
    // =========================================================================
    
    // Initialize quaternion directly from sensor readings
    // Call this ONCE at startup (or after reset) with a stable reading
    // This eliminates convergence delay entirely!
    void initialize_from_sensors(float ax, float ay, float az,
                                  float mx, float my, float mz);
    
    // Initialize from accelerometer only (6-DOF) - sets roll/pitch, yaw=0
    void initialize_from_accel(float ax, float ay, float az);
    
    // Set quaternion directly (if you have it from somewhere)
    void set_quaternion(float q0, float q1, float q2, float q3);

    // =========================================================================
    // ADAPTIVE BETA CONFIGURATION
    // =========================================================================
    
    // Enable/disable adaptive beta (motion-based adjustment)
    void set_adaptive_beta(bool enabled) { adaptive_beta_enabled_ = enabled; }
    bool get_adaptive_beta() const { return adaptive_beta_enabled_; }
    
    // Set beta range for adaptive mode
    // stationary_beta: used when device is still (higher = faster correction)
    // motion_beta: used when device is moving (lower = trust gyro more)
    void set_beta_range(float stationary_beta, float motion_beta);
    
    // Set gyro threshold for motion detection (in rad/s)
    // Default: 0.05 rad/s ≈ 3 deg/s
    void set_motion_threshold(float threshold) { motion_threshold_ = threshold; }
    
    // =========================================================================
    // FAST CONVERGENCE MODE
    // =========================================================================
    
    // Start fast convergence mode - uses high beta for initial alignment
    // duration_ms: how long to run in fast mode (0 = until stable)
    // fast_beta: beta to use during fast convergence (default 2.5)
    void start_fast_convergence(uint32_t duration_ms = 2000, float fast_beta = 2.5f);
    
    // Check if still in fast convergence mode
    bool in_fast_convergence() const { return fast_convergence_active_; }
    
    // Force exit fast convergence mode
    void end_fast_convergence();

    // =========================================================================
    // UPDATE METHODS (same as before, but now use adaptive beta internally)
    // =========================================================================
    
    // Update with 9-DOF sensor data (gyro in rad/s, accel in g, mag in uT)
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt);

    // Update with 6-DOF sensor data (no magnetometer)
    void update_imu(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float dt);

    // =========================================================================
    // OUTPUT METHODS
    // =========================================================================
    
    // Get quaternion
    void get_quaternion(float* q0, float* q1, float* q2, float* q3) const;
    const float* get_quaternion() const { return q_; }

    // Get Euler angles (in degrees)
    float get_roll() const;
    float get_pitch() const;
    float get_yaw() const;

    // Set filter gain (higher = faster but noisier)
    void set_beta(float beta) { base_beta_ = beta; beta_ = beta; }
    float get_beta() const { return beta_; }
    float get_base_beta() const { return base_beta_; }
    
    // Get the currently active beta (may differ from base if adaptive/fast mode)
    float get_active_beta() const { return beta_; }

    // Reset to identity orientation [1,0,0,0]
    void reset();
    
    // =========================================================================
    // DIAGNOSTICS
    // =========================================================================
    
    // Get convergence metric (how fast quaternion is changing)
    // Low value = stable/converged, High value = still converging
    float get_convergence_rate() const { return convergence_rate_; }
    
    // Check if filter is likely converged (heuristic)
    bool is_converged() const { return convergence_rate_ < 0.01f && !fast_convergence_active_; }

private:
    float q_[4];           // Quaternion [w, x, y, z]
    float q_prev_[4];      // Previous quaternion (for convergence detection)
    float beta_;           // Current active filter gain
    float base_beta_;      // Base/target beta for normal operation
    
    // Adaptive beta settings
    bool adaptive_beta_enabled_ = false;
    float stationary_beta_ = 0.5f;   // Beta when still
    float motion_beta_ = 0.05f;      // Beta when moving
    float motion_threshold_ = 0.05f; // Gyro magnitude threshold (rad/s)
    
    // Fast convergence mode
    bool fast_convergence_active_ = false;
    uint32_t fast_convergence_start_ = 0;
    uint32_t fast_convergence_duration_ = 0;
    float fast_beta_ = 2.5f;
    
    // Convergence tracking
    float convergence_rate_ = 1.0f;
    
    // Internal helper to compute effective beta
    float compute_effective_beta(float gyro_magnitude);
    
    // Update convergence tracking
    void update_convergence_tracking();

    // Fast inverse square root
    static float inv_sqrt(float x);
    
    // Helper: Euler to quaternion
    static void euler_to_quaternion(float roll, float pitch, float yaw,
                                     float* q0, float* q1, float* q2, float* q3);
};

#endif // MADGWICK_H
