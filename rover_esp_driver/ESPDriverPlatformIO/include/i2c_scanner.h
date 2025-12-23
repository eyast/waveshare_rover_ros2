#ifndef I2C_SCANNER_H
#define I2C_SCANNER_H

#include <Arduino.h>
#include <Wire.h>

// Scans all I2C addresses (0x01-0x7F) and prints found devices to Serial.
// Returns the number of devices found.
// Call Wire.begin() before using this function.
uint8_t scanI2CDevices();

#endif