#include "madgwick.h"
#include <math.h>

MadgwickFilter::MadgwickFilter(float beta) : base_beta_(beta), beta_(beta) {
    reset();
}

void MadgwickFilter::reset() {
    q_[0] = 1.0f;  // w
    q_[1] = 0.0f;  // x
    q_[2] = 0.0f;  // y
    q_[3] = 0.0f;  // z
    
    // Copy to previous for convergence tracking
    for (int i = 0; i < 4; i++) {
        q_prev_[i] = q_[i];
    }
    
    convergence_rate_ = 1.0f;  // Not converged yet
    fast_convergence_active_ = false;
}

// =============================================================================
// INITIALIZATION FROM SENSORS
// =============================================================================

void MadgwickFilter::initialize_from_accel(float ax, float ay, float az) {
    // Normalize accelerometer
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) {
        Serial.println("WARNING: Accel magnitude too low for initialization");
        return;
    }
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    // Compute roll and pitch from gravity vector
    // Roll: rotation about X-axis (phi)
    // Pitch: rotation about Y-axis (theta)
    // Yaw: set to 0 (no magnetometer)
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    float yaw = 0.0f;
    
    // Convert to quaternion
    euler_to_quaternion(roll, pitch, yaw, &q_[0], &q_[1], &q_[2], &q_[3]);
    
    // Copy to previous
    for (int i = 0; i < 4; i++) {
        q_prev_[i] = q_[i];
    }
    
    convergence_rate_ = 0.1f;  // Mostly converged
    
    Serial.print("Initialized from accel: R=");
    Serial.print(roll * 180.0f / PI, 1);
    Serial.print(" P=");
    Serial.print(pitch * 180.0f / PI, 1);
    Serial.println(" Y=0");
}

void MadgwickFilter::initialize_from_sensors(float ax, float ay, float az,
                                              float mx, float my, float mz) {
    // Normalize accelerometer
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) {
        Serial.println("WARNING: Accel magnitude too low for initialization");
        return;
    }
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    // Compute roll and pitch from gravity
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    
    // Tilt-compensate magnetometer to get horizontal components
    float cos_roll = cosf(roll);
    float sin_roll = sinf(roll);
    float cos_pitch = cosf(pitch);
    float sin_pitch = sinf(pitch);
    
    // Rotate magnetometer reading into horizontal plane
    // mx_h and my_h are the horizontal components of the magnetic field
    float mx_h = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
    float my_h = my * cos_roll - mz * sin_roll;
    
    // Compute yaw from horizontal magnetic field
    // Note: atan2(-my, mx) gives heading where 0 = magnetic north
    float yaw = atan2f(-my_h, mx_h);
    
    // Convert to quaternion
    euler_to_quaternion(roll, pitch, yaw, &q_[0], &q_[1], &q_[2], &q_[3]);
    
    // Copy to previous
    for (int i = 0; i < 4; i++) {
        q_prev_[i] = q_[i];
    }
    
    convergence_rate_ = 0.05f;  // Well converged
    
    Serial.print("Initialized from sensors: R=");
    Serial.print(roll * 180.0f / PI, 1);
    Serial.print(" P=");
    Serial.print(pitch * 180.0f / PI, 1);
    Serial.print(" Y=");
    Serial.println(yaw * 180.0f / PI, 1);
}

void MadgwickFilter::set_quaternion(float q0, float q1, float q2, float q3) {
    // Normalize
    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 0.001f) {
        q_[0] = q0 / norm;
        q_[1] = q1 / norm;
        q_[2] = q2 / norm;
        q_[3] = q3 / norm;
    }
    
    for (int i = 0; i < 4; i++) {
        q_prev_[i] = q_[i];
    }
    
    convergence_rate_ = 0.05f;
}

void MadgwickFilter::euler_to_quaternion(float roll, float pitch, float yaw,
                                          float* q0, float* q1, float* q2, float* q3) {
    // Convert Euler angles (in radians) to quaternion
    // Using ZYX convention (yaw, pitch, roll)
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    
    *q0 = cr * cp * cy + sr * sp * sy;  // w
    *q1 = sr * cp * cy - cr * sp * sy;  // x
    *q2 = cr * sp * cy + sr * cp * sy;  // y
    *q3 = cr * cp * sy - sr * sp * cy;  // z
}

// =============================================================================
// ADAPTIVE BETA
// =============================================================================

void MadgwickFilter::set_beta_range(float stationary_beta, float motion_beta) {
    stationary_beta_ = stationary_beta;
    motion_beta_ = motion_beta;
}

float MadgwickFilter::compute_effective_beta(float gyro_magnitude) {
    // Handle fast convergence mode first
    if (fast_convergence_active_) {
        uint32_t elapsed = millis() - fast_convergence_start_;
        
        if (fast_convergence_duration_ > 0 && elapsed >= fast_convergence_duration_) {
            // Time-based exit
            end_fast_convergence();
        } else if (convergence_rate_ < 0.005f) {
            // Stability-based exit
            end_fast_convergence();
        } else {
            // Still in fast mode - exponentially decay from fast_beta to base_beta
            // Over the duration, go from fast_beta to base_beta
            float progress = (float)elapsed / (float)fast_convergence_duration_;
            if (progress > 1.0f) progress = 1.0f;
            
            // Exponential decay: beta = fast_beta * exp(-k*t) + base_beta * (1 - exp(-k*t))
            // Simplified: linear blend for now
            float decay = expf(-3.0f * progress);  // ~95% decay at progress=1.0
            return fast_beta_ * decay + base_beta_ * (1.0f - decay);
        }
    }
    
    // Adaptive beta based on motion
    if (adaptive_beta_enabled_) {
        if (gyro_magnitude < motion_threshold_) {
            // Stationary - trust accelerometer/magnetometer more
            return stationary_beta_;
        } else {
            // Moving - trust gyroscope more
            // Smooth transition: interpolate based on how much over threshold
            float motion_factor = (gyro_magnitude - motion_threshold_) / motion_threshold_;
            if (motion_factor > 1.0f) motion_factor = 1.0f;
            return stationary_beta_ * (1.0f - motion_factor) + motion_beta_ * motion_factor;
        }
    }
    
    return base_beta_;
}

// =============================================================================
// FAST CONVERGENCE
// =============================================================================

void MadgwickFilter::start_fast_convergence(uint32_t duration_ms, float fast_beta) {
    fast_convergence_active_ = true;
    fast_convergence_start_ = millis();
    fast_convergence_duration_ = duration_ms;
    fast_beta_ = fast_beta;
    
    Serial.print("Fast convergence started: beta=");
    Serial.print(fast_beta);
    Serial.print(" duration=");
    Serial.print(duration_ms);
    Serial.println("ms");
}

void MadgwickFilter::end_fast_convergence() {
    if (fast_convergence_active_) {
        fast_convergence_active_ = false;
        beta_ = base_beta_;
        
        Serial.print("Fast convergence ended. Final orientation: Y=");
        Serial.print(get_yaw(), 1);
        Serial.print(" P=");
        Serial.print(get_pitch(), 1);
        Serial.print(" R=");
        Serial.println(get_roll(), 1);
    }
}

// =============================================================================
// CONVERGENCE TRACKING
// =============================================================================

void MadgwickFilter::update_convergence_tracking() {
    // Compute quaternion change rate
    float dq = 0;
    for (int i = 0; i < 4; i++) {
        float diff = q_[i] - q_prev_[i];
        dq += diff * diff;
        q_prev_[i] = q_[i];
    }
    
    // Low-pass filter the convergence rate
    float new_rate = sqrtf(dq);
    convergence_rate_ = 0.9f * convergence_rate_ + 0.1f * new_rate;
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

float MadgwickFilter::inv_sqrt(float x) {
    return 1.0f / sqrtf(x);
}

// =============================================================================
// UPDATE WITH 9-DOF (Accel + Gyro + Mag)
// =============================================================================

void MadgwickFilter::update(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float mx, float my, float mz,
                            float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz;
    float _4bx, _4bz, _2q0, _2q1, _2q2, _2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
    
    // Compute effective beta based on motion and convergence state
    float gyro_magnitude = sqrtf(gx*gx + gy*gy + gz*gz);
    beta_ = compute_effective_beta(gyro_magnitude);

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalize accelerometer measurement
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalize magnetometer measurement
        recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 +
             _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 -
             my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 -
               mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient descent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0 * q2 - ax) +
             _2q1 * (2.0f * q0q1 + _2q2 * q3 - ay) -
             _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        s1 = _2q3 * (2.0f * q1q3 - _2q0 * q2 - ax) +
             _2q0 * (2.0f * q0q1 + _2q2 * q3 - ay) -
             4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
             _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        s2 = -_2q0 * (2.0f * q1q3 - _2q0 * q2 - ax) +
             _2q3 * (2.0f * q0q1 + _2q2 * q3 - ay) -
             4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
             (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        s3 = _2q1 * (2.0f * q1q3 - _2q0 * q2 - ax) +
             _2q2 * (2.0f * q0q1 + _2q2 * q3 - ay) +
             (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        // Normalize step magnitude
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta_ * s0;
        qDot2 -= beta_ * s1;
        qDot3 -= beta_ * s2;
        qDot4 -= beta_ * s3;
    }

    // Integrate rate of change of quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalize quaternion
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q_[0] = q0 * recipNorm;
    q_[1] = q1 * recipNorm;
    q_[2] = q2 * recipNorm;
    q_[3] = q3 * recipNorm;
    
    // Track convergence
    update_convergence_tracking();
}

// =============================================================================
// UPDATE WITH 6-DOF (Accel + Gyro only)
// =============================================================================

void MadgwickFilter::update_imu(float gx, float gy, float gz,
                                 float ax, float ay, float az,
                                 float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;

    float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
    
    // Compute effective beta
    float gyro_magnitude = sqrtf(gx*gx + gy*gy + gz*gz);
    beta_ = compute_effective_beta(gyro_magnitude);

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalize accelerometer measurement
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;

        // Gradient descent corrective step
        s0 = _4q0 * q2 * q2 + _2q2 * ax + _4q0 * q1 * q1 - _2q1 * ay;
        s1 = _4q1 * q3 * q3 - _2q3 * ax + 4.0f * q0 * q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1 * q1 + _8q1 * q2 * q2 + _4q1 * az;
        s2 = 4.0f * q0 * q0 * q2 + _2q0 * ax + _4q2 * q3 * q3 - _2q3 * ay - _4q2 + _8q2 * q1 * q1 + _8q2 * q2 * q2 + _4q2 * az;
        s3 = 4.0f * q1 * q1 * q3 - _2q1 * ax + 4.0f * q2 * q2 * q3 - _2q2 * ay;

        // Normalize step magnitude
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta_ * s0;
        qDot2 -= beta_ * s1;
        qDot3 -= beta_ * s2;
        qDot4 -= beta_ * s3;
    }

    // Integrate rate of change of quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalize quaternion
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q_[0] = q0 * recipNorm;
    q_[1] = q1 * recipNorm;
    q_[2] = q2 * recipNorm;
    q_[3] = q3 * recipNorm;
    
    // Track convergence
    update_convergence_tracking();
}

// =============================================================================
// OUTPUT METHODS
// =============================================================================

void MadgwickFilter::get_quaternion(float* q0, float* q1, float* q2, float* q3) const {
    *q0 = q_[0];
    *q1 = q_[1];
    *q2 = q_[2];
    *q3 = q_[3];
}

float MadgwickFilter::get_roll() const {
    float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
    return atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
}

float MadgwickFilter::get_pitch() const {
    float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    // Handle gimbal lock
    if (fabsf(sinp) >= 1.0f) {
        return copysignf(90.0f, sinp);
    }
    return asinf(sinp) * 180.0f / PI;
}

float MadgwickFilter::get_yaw() const {
    float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
    return atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;
}
