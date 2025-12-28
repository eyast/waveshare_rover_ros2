/*
 * OLED Display Module
 * 
 * Simple status display showing battery info.
 * Refreshes periodically based on configurable interval.
 */

#ifndef OLED_H
#define OLED_H

#include <Arduino.h>
#include "config.h"

// =============================================================================
// OLED Configuration
// =============================================================================

#define OLED_WIDTH          128
#define OLED_HEIGHT         32
#define OLED_ADDR           0x3C
#define OLED_RESET          -1

// Refresh interval - adjust this value as needed (milliseconds)
// Default: 60000 = 1 minute
extern uint32_t oled_refresh_interval_ms;

// FreeRTOS task config
#define OLED_TASK_STACK     2048
#define OLED_TASK_PRIORITY  1       // Low priority

// =============================================================================
// OLED Functions
// =============================================================================

// Initialize OLED and start display task
void oled_init();

// Set data to display (called by power task)
void oled_set_power(float voltage_V, float current_mA, float power_mW);

// Status
bool oled_is_ok();

#endif // OLED_H