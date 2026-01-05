#ifndef I2C_HELPERS_H
#define I2C_HELPERS_H

#include <Arduino.h>
#include <Wire.h>
#include "syshelpers.h"

// =============================================================================
// I2C HELPER FUNCTIONS
// =============================================================================
// WHY: I2C is complex. These functions hide the complexity and provide simple
//      interfaces. They handle error checking and return clear success/fail.
//
// WHAT IS I2C: A 2-wire communication bus that lets multiple devices share
//              the same wires. Like a party line phone - devices take turns.

// Initialize the I2C bus
// WHEN: Call once in setup() before using any I2C devices
// WHY: Sets up the wire connections and communication speed
void i2c_init(uint8_t sda_pin, uint8_t scl_pin, uint32_t frequency);

// Write a single byte to a device register
// PARAMS:
//   device_addr: Which device on the bus (like a street address)
//   register_addr: Which setting in that device (like an apartment number)
//   value: The byte to write
// RETURNS: true if successful, false if device didn't respond
bool i2c_write_byte(uint8_t device_addr, uint8_t register_addr, uint8_t value);

// Read a single byte from a device register
// PARAMS:
//   device_addr: Which device to read from
//   register_addr: Which register to read
// RETURNS: The byte read, or 0xFF if error
// WHY 0xFF for error: It's unlikely to be a real value, signals problem
uint8_t i2c_read_byte(uint8_t device_addr, uint8_t register_addr);

// Read multiple bytes starting from a register
// PARAMS:
//   device_addr: Which device
//   start_register: Where to start reading
//   buffer: Where to store the bytes we read
//   count: How many bytes to read
// RETURNS: true if successful, false if communication failed
// WHY: Reading multiple bytes at once is faster than one at a time
bool i2c_read_bytes(uint8_t device_addr, uint8_t start_register, 
                    uint8_t* buffer, uint8_t count);

#endif // I2C_HELPERS_H
