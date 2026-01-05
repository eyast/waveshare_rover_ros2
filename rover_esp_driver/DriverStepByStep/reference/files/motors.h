#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// =============================================================================
// MOTOR CONTROL CLASS
// =============================================================================
// WHY: Abstracts the complexity of H-bridge motor control
// WHAT: H-bridge allows controlling direction and speed of DC motors
// WHAT IF no abstraction: Would need to remember pin combinations everywhere

class Motors {
public:
    Motors();
    
    // Initialize motor hardware
    // WHEN: Call once in setup()
    // WHAT: Configures GPIO pins and PWM channels
    void begin();
    
    // Set motor speeds
    // PARAMS:
    //   left_speed: -255 to +255 (negative = reverse)
    //   right_speed: -255 to +255
    // WHY ±255: Matches 8-bit PWM resolution (0-255)
    void set_speeds(int16_t left_speed, int16_t right_speed);
    
    // Stop both motors immediately
    // WHY separate function: Emergency stop is common operation
    void stop();
    
    // Set individual motor
    // WHY: Sometimes need to control motors independently
    void set_left(int16_t speed);
    void set_right(int16_t speed);
    
private:
    // Helper to set one motor's speed
    void set_motor(bool is_left, int16_t speed);
};

// =============================================================================
// GLOBAL INSTANCE
// =============================================================================
extern Motors motors;

#endif // MOTORS_H
