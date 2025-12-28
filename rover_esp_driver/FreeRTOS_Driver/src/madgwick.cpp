/*
 * Madgwick AHRS Filter Implementation
 */

#include "madgwick.h"
#include <math.h>

// Global instance
MadgwickFilter filter(FILTER_BETA_DEFAULT);

MadgwickFilter::MadgwickFilter(float beta) : base_beta_(beta), beta_(beta) {
    adaptive_enabled_ = FILTER_ADAPTIVE_ENABLED;
    stat_beta_ = FILTER_BETA_STATIONARY;
    motion_beta_ = FILTER_BETA_MOTION;
    motion_threshold_ = FILTER_MOTION_THRESHOLD;
    fast_active_ = false;
    reset();
}

void MadgwickFilter::reset() {
    q_[0] = 1.0f;  // w
    q_[1] = 0.0f;  // x
    q_[2] = 0.0f;  // y
    q_[3] = 0.0f;  // z
    
    for (int i = 0; i < 4; i++) {
        q_prev_[i] = q_[i];
    }
    
    conv_rate_ = 1.0f;
    fast_active_ = false;
}

// =============================================================================
// Initialization
// =============================================================================

void MadgwickFilter::euler_to_quat(float roll, float pitch, float yaw, float* q) {
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    
    q[0] = cr * cp * cy + sr * sp * sy;  // w
    q[1] = sr * cp * cy - cr * sp * sy;  // x
    q[2] = cr * sp * cy + sr * cp * sy;  // y
    q[3] = cr * cp * sy - sr * sp * cy;  // z
}

void MadgwickFilter::init_from_accel(float ax, float ay, float az) {
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) return;
    
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    float yaw = 0.0f;
    
    euler_to_quat(roll, pitch, yaw, q_);
    
    for (int i = 0; i < 4; i++) {
        q_prev_[i] = q_[i];
    }
    
    conv_rate_ = 0.1f;
}

void MadgwickFilter::init_from_sensors(float ax, float ay, float az,
                                        float mx, float my, float mz) {
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) return;
    
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    // Roll and pitch from gravity
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    
    // Tilt-compensate magnetometer
    float cos_r = cosf(roll);
    float sin_r = sinf(roll);
    float cos_p = cosf(pitch);
    float sin_p = sinf(pitch);
    
    float mx_h = mx * cos_p + my * sin_r * sin_p + mz * cos_r * sin_p;
    float my_h = my * cos_r - mz * sin_r;
    
    // Yaw from horizontal magnetic field
    float yaw = atan2f(-my_h, mx_h);
    
    euler_to_quat(roll, pitch, yaw, q_);
    
    for (int i = 0; i < 4; i++) {
        q_prev_[i] = q_[i];
    }
    
    conv_rate_ = 0.05f;
}

// =============================================================================
// Adaptive Beta
// =============================================================================

void MadgwickFilter::set_beta_range(float stationary, float motion) {
    stat_beta_ = stationary;
    motion_beta_ = motion;
}

float MadgwickFilter::compute_beta(float gyro_mag) {
    // Fast convergence mode
    if (fast_active_) {
        uint32_t elapsed = millis() - fast_start_;
        
        if (fast_duration_ > 0 && elapsed >= fast_duration_) {
            end_fast_convergence();
        } else if (conv_rate_ < 0.005f) {
            end_fast_convergence();
        } else {
            float progress = (float)elapsed / (float)fast_duration_;
            if (progress > 1.0f) progress = 1.0f;
            float decay = expf(-3.0f * progress);
            return fast_beta_ * decay + base_beta_ * (1.0f - decay);
        }
    }
    
    // Adaptive beta based on motion
    if (adaptive_enabled_) {
        if (gyro_mag < motion_threshold_) {
            return stat_beta_;
        } else {
            float factor = (gyro_mag - motion_threshold_) / motion_threshold_;
            if (factor > 1.0f) factor = 1.0f;
            return stat_beta_ * (1.0f - factor) + motion_beta_ * factor;
        }
    }
    
    return base_beta_;
}

// =============================================================================
// Fast Convergence
// =============================================================================

void MadgwickFilter::start_fast_convergence(uint32_t duration_ms, float beta) {
    fast_active_ = true;
    fast_start_ = millis();
    fast_duration_ = duration_ms;
    fast_beta_ = beta;
}

void MadgwickFilter::end_fast_convergence() {
    if (fast_active_) {
        fast_active_ = false;
        beta_ = base_beta_;
    }
}

// =============================================================================
// Convergence Tracking
// =============================================================================

void MadgwickFilter::update_convergence() {
    float dq = 0;
    for (int i = 0; i < 4; i++) {
        float diff = q_[i] - q_prev_[i];
        dq += diff * diff;
        q_prev_[i] = q_[i];
    }
    
    float new_rate = sqrtf(dq);
    conv_rate_ = 0.9f * conv_rate_ + 0.1f * new_rate;
}

// =============================================================================
// Utility
// =============================================================================

float MadgwickFilter::inv_sqrt(float x) {
    return 1.0f / sqrtf(x);
}

// =============================================================================
// Update with 9-DOF
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
    
    float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);
    beta_ = compute_beta(gyro_mag);

    // Rate of change from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalize accelerometer
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalize magnetometer
        recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables
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

        // Earth's magnetic field reference
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 +
             _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 -
             my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 -
               mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient descent step
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

        // Normalize step
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback
        qDot1 -= beta_ * s0;
        qDot2 -= beta_ * s1;
        qDot3 -= beta_ * s2;
        qDot4 -= beta_ * s3;
    }

    // Integrate
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
    
    update_convergence();
}

// =============================================================================
// Update with 6-DOF
// =============================================================================

void MadgwickFilter::update_imu(float gx, float gy, float gz,
                                  float ax, float ay, float az,
                                  float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;

    float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
    
    float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);
    beta_ = compute_beta(gyro_mag);

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;

        s0 = _4q0 * q2 * q2 + _2q2 * ax + _4q0 * q1 * q1 - _2q1 * ay;
        s1 = _4q1 * q3 * q3 - _2q3 * ax + 4.0f * q0 * q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1 * q1 + _8q1 * q2 * q2 + _4q1 * az;
        s2 = 4.0f * q0 * q0 * q2 + _2q0 * ax + _4q2 * q3 * q3 - _2q3 * ay - _4q2 + _8q2 * q1 * q1 + _8q2 * q2 * q2 + _4q2 * az;
        s3 = 4.0f * q1 * q1 * q3 - _2q1 * ax + 4.0f * q2 * q2 * q3 - _2q2 * ay;

        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        qDot1 -= beta_ * s0;
        qDot2 -= beta_ * s1;
        qDot3 -= beta_ * s2;
        qDot4 -= beta_ * s3;
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q_[0] = q0 * recipNorm;
    q_[1] = q1 * recipNorm;
    q_[2] = q2 * recipNorm;
    q_[3] = q3 * recipNorm;
    
    update_convergence();
}

// =============================================================================
// Output Methods
// =============================================================================

void MadgwickFilter::get_quaternion(float* q0, float* q1, float* q2, float* q3) const {
    *q0 = q_[0];
    *q1 = q_[1];
    *q2 = q_[2];
    *q3 = q_[3];
}

float MadgwickFilter::roll() const {
    return atan2f(2.0f * (q_[0] * q_[1] + q_[2] * q_[3]),
                  1.0f - 2.0f * (q_[1] * q_[1] + q_[2] * q_[2])) * 180.0f / PI;
}

float MadgwickFilter::pitch() const {
    float sinp = 2.0f * (q_[0] * q_[2] - q_[3] * q_[1]);
    if (fabsf(sinp) >= 1.0f) {
        return copysignf(90.0f, sinp);
    }
    return asinf(sinp) * 180.0f / PI;
}

float MadgwickFilter::yaw() const {
    return atan2f(2.0f * (q_[0] * q_[3] + q_[1] * q_[2]),
                  1.0f - 2.0f * (q_[2] * q_[2] + q_[3] * q_[3])) * 180.0f / PI;
}
