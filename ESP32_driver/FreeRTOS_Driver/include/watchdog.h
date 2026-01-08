/*
 * Watchdog Health Monitoring Module
 * 
 * Monitors FreeRTOS tasks and ensures they're responding.
 * Triggers system reset if critical tasks hang.
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"

// =============================================================================
// Task Identifiers
// =============================================================================

enum class TaskID : int {
    IMU = 0,
    TELEMETRY = 1,
    COMMAND = 2,
    POWER = 3,
    TASK_COUNT = 4
};

// =============================================================================
// Public API
// =============================================================================

// Initialize watchdog system (call once during setup)
void watchdog_init();

// Create and start the watchdog monitoring task
// Returns task handle (can be NULL if creation fails)
TaskHandle_t watchdog_start_task();

// Register a task handle for stack monitoring
// Call this after creating each task you want to monitor
void watchdog_register_task(TaskID task_id, TaskHandle_t handle);

// Report that a task is alive (call from each task's main loop)
void watchdog_heartbeat(TaskID task_id);

// Check if watchdog is ready to accept heartbeats
bool watchdog_is_ready();

// Enable/disable monitoring for a specific task
void watchdog_enable_task(TaskID task_id, bool enabled);

// Get statistics for a task
struct TaskStats {
    const char* name;
    uint32_t last_seen_ms_ago;
    uint32_t failure_count;
    bool enabled;
    bool alive;
};

TaskStats watchdog_get_stats(TaskID task_id);

// Print health report to serial
void watchdog_print_status();

// Print Stack info
void watchdog_print_stack_usage();

#endif // WATCHDOG_H