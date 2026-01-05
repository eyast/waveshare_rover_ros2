#include "motors.h"
#include "config.h"

// =============================================================================
// GLOBAL INSTANCE
// =============================================================================

Motors motors;

// =============================================================================
// CONSTRUCTOR
// =============================================================================

Motors::Motors() {
    // Nothing to initialize here - done in begin()
}

// =============================================================================
// INITIALIZATION
// =============================================================================

void Motors::begin() {
    Serial.println("\n=== Initializing Motors ===");
    
    // STEP 1: Configure GPIO pins as outputs
    // WHY: Must tell ESP32 which pins we'll control
    // WHAT: Direction control pins (IN1, IN2)
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    
    // STEP 2: Set initial state - all low (motors off)
    // WHY: Safe starting state prevents unexpected movement
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
    
    // STEP 3: Configure PWM channels
    // WHY PWM: Variable speed control
    // WHAT: ESP32 has 16 independent PWM channels - we use 2
    
    // Setup Motor A PWM
    // PARAMS: channel, frequency, resolution
    ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    // Attach channel to GPIO pin
    ledcAttachPin(MOTOR_A_PWM, PWM_CHANNEL_A);
    
    // Setup Motor B PWM
    ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_B_PWM, PWM_CHANNEL_B);
    
    // STEP 4: Set PWM to 0 (motors stopped)
    // WHY: Ensures motors don't run until commanded
    ledcWrite(PWM_CHANNEL_A, 0);
    ledcWrite(PWM_CHANNEL_B, 0);
    
    Serial.println("[Motors] Initialized");
}

// =============================================================================
// MOTOR CONTROL
// =============================================================================

void Motors::set_speeds(int16_t left_speed, int16_t right_speed) {
    set_left(left_speed);
    set_right(right_speed);
}

void Motors::set_left(int16_t speed) {
    set_motor(true, speed);  // true = left motor
}

void Motors::set_right(int16_t speed) {
    set_motor(false, speed);  // false = right motor
}

void Motors::stop() {
    set_speeds(0, 0);
}

void Motors::set_motor(bool is_left, int16_t speed) {
    // STEP 1: Clamp speed to valid range
    // WHY: Prevent invalid values that could cause problems
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    
    // STEP 2: Get the pins for this motor
    // WHY: Different motors use different pins
    uint8_t pwm_pin, in1_pin, in2_pin, pwm_channel;
    
    if (is_left) {
        // Motor A (left)
        pwm_pin = MOTOR_A_PWM;
        in1_pin = MOTOR_A_IN1;
        in2_pin = MOTOR_A_IN2;
        pwm_channel = PWM_CHANNEL_A;
    } else {
        // Motor B (right)
        pwm_pin = MOTOR_B_PWM;
        in1_pin = MOTOR_B_IN1;
        in2_pin = MOTOR_B_IN2;
        pwm_channel = PWM_CHANNEL_B;
    }
    
    // STEP 3: Handle direction
    // HOW H-BRIDGE WORKS:
    //   IN1=LOW,  IN2=LOW  → Motor off (brake)
    //   IN1=HIGH, IN2=LOW  → Motor forward
    //   IN1=LOW,  IN2=HIGH → Motor reverse
    //   IN1=HIGH, IN2=HIGH → Motor off (not used)
    
    if (speed == 0) {
        // Stop: Both pins LOW
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, LOW);
        ledcWrite(pwm_channel, 0);
    }
    else if (speed > 0) {
        // Forward: IN1=LOW, IN2=HIGH
        // WHY this way: Matches our wiring convention
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
        ledcWrite(pwm_channel, speed);  // Set speed
    }
    else {  // speed < 0
        // Reverse: IN1=HIGH, IN2=LOW
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
        ledcWrite(pwm_channel, -speed);  // Make positive for PWM
    }
}

// =============================================================================
// HOW THIS WORKS: H-BRIDGE MOTOR DRIVER EXPLAINED
// =============================================================================
//
// WHAT IS AN H-BRIDGE:
// An H-bridge is an electronic circuit that allows a voltage to be applied
// across a load in either direction. Called "H-bridge" because the circuit
// diagram looks like the letter H.
//
//         +V
//          |
//      [S1]   [S2]
//       |       |
//       +---M---+    M = Motor
//       |       |
//      [S3]   [S4]
//          |
//         GND
//
// S1-S4 are switches (transistors). The motor is in the middle.
//
// OPERATION MODES:
//
// 1. FORWARD:
//    S1=ON, S4=ON, S2=OFF, S3=OFF
//    Current flows: +V → S1 → Motor → S4 → GND
//    Motor spins forward
//
// 2. REVERSE:
//    S2=ON, S3=ON, S1=OFF, S4=OFF
//    Current flows: +V → S2 → Motor → S3 → GND
//    Motor spins backward (opposite direction)
//
// 3. BRAKE:
//    All switches OFF, or both bottom switches ON
//    Motor stopped, can't spin freely
//
// 4. COAST:
//    All switches OFF
//    Motor stopped but can spin freely
//
// PWM FOR SPEED CONTROL:
// We switch the H-bridge on/off rapidly (20,000 times per second).
// The percentage of "on" time controls the average voltage → speed.
//
// Example:
//   100% duty cycle = full speed (always on)
//   50% duty cycle = half speed (on half the time)
//   0% duty cycle = stopped (always off)
//
// WHY 20kHz:
//   - Above human hearing (won't hear whine)
//   - Fast enough for smooth motor response
//   - Slow enough for ESP32 to handle easily
//
// =============================================================================
