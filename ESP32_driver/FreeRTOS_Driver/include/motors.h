/*
 * Motor Control Module
 * 
 * Simple motor control without encoders.
 * Provides direct PWM control with safety timeout.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "config.h"

// Motor state
struct MotorState {
    int16_t speed_left;     // -255 to +255
    int16_t speed_right;    // -255 to +255
    uint32_t last_cmd_time; // Last command timestamp
    bool enabled;           // Motors enabled
    bool timeout_stop;      // Stopped due to timeout
};

extern MotorState motors;

// =============================================================================
// Initialization
// =============================================================================

void motors_init();

// =============================================================================
// Control Functions
// =============================================================================

// Set motor speeds (-255 to +255, negative = reverse)
void motors_set(int16_t left, int16_t right);

// Stop motors immediately
void motors_stop();

// Emergency stop (cut power, no gradual stop)
void motors_emergency_stop();

// Enable/disable motor output
void motors_enable(bool enable);

// =============================================================================
// Safety
// =============================================================================

// Check for timeout and stop motors if no command received
// Call this periodically (e.g., in a FreeRTOS task)
void motors_check_timeout();

// Reset the command timeout timer
void motors_heartbeat();

// =============================================================================
// Status
// =============================================================================

int16_t motors_get_left();
int16_t motors_get_right();
bool motors_is_enabled();
bool motors_is_timeout();

#endif // MOTORS_H
