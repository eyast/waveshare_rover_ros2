/*
 * Motor Control Module Implementation
 */

#include "motors.h"
#include "protocol.h"

// Global state
MotorState motors = {
    .speed_left = 0,
    .speed_right = 0,
    .last_cmd_time = 0,
    .enabled = true,
    .timeout_stop = false
};

// =============================================================================
// Initialization
// =============================================================================

void motors_init() {
    // Configure GPIO
    pinMode(PIN_MOTOR_A_IN1, OUTPUT);
    pinMode(PIN_MOTOR_A_IN2, OUTPUT);
    pinMode(PIN_MOTOR_A_PWM, OUTPUT);
    pinMode(PIN_MOTOR_B_IN1, OUTPUT);
    pinMode(PIN_MOTOR_B_IN2, OUTPUT);
    pinMode(PIN_MOTOR_B_PWM, OUTPUT);
    
    // Configure PWM channels
    ledcSetup(MOTOR_PWM_CHANNEL_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcAttachPin(PIN_MOTOR_A_PWM, MOTOR_PWM_CHANNEL_A);
    
    ledcSetup(MOTOR_PWM_CHANNEL_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcAttachPin(PIN_MOTOR_B_PWM, MOTOR_PWM_CHANNEL_B);
    
    // Initialize to stopped
    motors_stop();
    motors.last_cmd_time = millis();
    
    out_system("MOTORS", "OK");
}

// =============================================================================
// Internal Motor Control
// =============================================================================

static void set_motor_a(int16_t speed) {
    if (!motors.enabled) {
        speed = 0;
    }
    
    // Apply deadband
    if (abs(speed) < MOTOR_DEADBAND) {
        speed = 0;
    }
    
    // Clamp speed
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    
    if (speed == 0) {
        // Coast/brake
        digitalWrite(PIN_MOTOR_A_IN1, LOW);
        digitalWrite(PIN_MOTOR_A_IN2, LOW);
        ledcWrite(MOTOR_PWM_CHANNEL_A, 0);
    } else if (speed > 0) {
        // Forward
        digitalWrite(PIN_MOTOR_A_IN1, HIGH);
        digitalWrite(PIN_MOTOR_A_IN2, LOW);
        ledcWrite(MOTOR_PWM_CHANNEL_A, abs(speed));
    } else {
        // Reverse
        digitalWrite(PIN_MOTOR_A_IN1, LOW);
        digitalWrite(PIN_MOTOR_A_IN2, HIGH);
        ledcWrite(MOTOR_PWM_CHANNEL_A, abs(speed));
    }
}

static void set_motor_b(int16_t speed) {
    if (!motors.enabled) {
        speed = 0;
    }
    
    // Apply deadband
    if (abs(speed) < MOTOR_DEADBAND) {
        speed = 0;
    }
    
    // Clamp speed
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    
    if (speed == 0) {
        // Coast/brake
        digitalWrite(PIN_MOTOR_B_IN1, LOW);
        digitalWrite(PIN_MOTOR_B_IN2, LOW);
        ledcWrite(MOTOR_PWM_CHANNEL_B, 0);
    } else if (speed > 0) {
        // Forward
        digitalWrite(PIN_MOTOR_B_IN1, HIGH);
        digitalWrite(PIN_MOTOR_B_IN2, LOW);
        ledcWrite(MOTOR_PWM_CHANNEL_B, abs(speed));
    } else {
        // Reverse
        digitalWrite(PIN_MOTOR_B_IN1, LOW);
        digitalWrite(PIN_MOTOR_B_IN2, HIGH);
        ledcWrite(MOTOR_PWM_CHANNEL_B, abs(speed));
    }
}

// =============================================================================
// Control Functions
// =============================================================================

void motors_set(int16_t left, int16_t right) {
    motors.speed_left = left;
    motors.speed_right = right;
    motors.last_cmd_time = millis();
    motors.timeout_stop = false;
    
    set_motor_a(left);
    set_motor_b(right);
}

void motors_stop() {
    motors.speed_left = 0;
    motors.speed_right = 0;
    
    set_motor_a(0);
    set_motor_b(0);
}

void motors_emergency_stop() {
    // Cut all power immediately
    digitalWrite(PIN_MOTOR_A_IN1, LOW);
    digitalWrite(PIN_MOTOR_A_IN2, LOW);
    digitalWrite(PIN_MOTOR_B_IN1, LOW);
    digitalWrite(PIN_MOTOR_B_IN2, LOW);
    ledcWrite(MOTOR_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_PWM_CHANNEL_B, 0);
    
    motors.speed_left = 0;
    motors.speed_right = 0;
    motors.enabled = false;
}

void motors_enable(bool enable) {
    motors.enabled = enable;
    if (!enable) {
        motors_stop();
    }
}

// =============================================================================
// Safety
// =============================================================================

void motors_check_timeout() {
    if (!motors.enabled) return;
    
    uint32_t now = millis();
    if ((now - motors.last_cmd_time) > MOTOR_TIMEOUT_MS) {
        if (!motors.timeout_stop && (motors.speed_left != 0 || motors.speed_right != 0)) {
            motors_stop();
            motors.timeout_stop = true;
            out_system("MOTORS", "timeout_stop");
        }
    }
}

void motors_heartbeat() {
    motors.last_cmd_time = millis();
    motors.timeout_stop = false;
}

// =============================================================================
// Status
// =============================================================================

int16_t motors_get_left() {
    return motors.speed_left;
}

int16_t motors_get_right() {
    return motors.speed_right;
}

bool motors_is_enabled() {
    return motors.enabled;
}

bool motors_is_timeout() {
    return motors.timeout_stop;
}
