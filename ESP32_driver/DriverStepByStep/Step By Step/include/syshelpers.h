#ifndef SYSHELPERS_H
#define SYSHELPERS_H

#include <Arduino.h>

void start_serial(uint32_t baud);
// Starts Serial port

void sysecho(const char* message);
// Helper function to append Syste ID to all outgoing messages

void sysecho(const char* message, u_int8_t value);

uint16_t register_to_16u(uint8_t low, uint8_t high);

int16_t register_to_16(uint8_t low, uint8_t high);

#endif