/*
 * I2C Helper Functions
 * 
 * Simple wrapper functions for I2C bus operations.
 */

#ifndef I2C_HELPERS_H
#define I2C_HELPERS_H

#include <Arduino.h>
#include <Wire.h>

// Initialize I2C bus with configured pins
void i2c_init(uint8_t sda, uint8_t scl, uint32_t freq);

// Write a single byte to a register
bool i2c_write_register(uint8_t addr, uint8_t reg, uint8_t value);

// Read a single byte from a register
uint8_t i2c_read_register(uint8_t addr, uint8_t reg);

// Read multiple bytes starting from a register
bool i2c_read_registers(uint8_t addr, uint8_t start_reg, uint8_t* buffer, uint8_t count);

#endif // I2C_HELPERS_H
