/*
 * Watchdog Health Monitoring Implementation
 */

#include "watchdog.h"
#include "protocol.h"
#include "esp_task_wdt.h"

// =============================================================================
// Internal State
// =============================================================================

struct TaskHealth {
    const char* name;
    volatile uint32_t last_heartbeat_ms;
    uint32_t timeout_ms;
    bool enabled;
    volatile uint32_t failure_count;
    TaskHandle_t handle;
    uint32_t stack_size;
};

// Task health tracking array
static TaskHealth task_health[4] = {
    {"IMU",       0, WDT_TIMEOUT_IMU_MS,     true, 0, NULL, STACK_SIZE_IMU * 4},
    {"Telemetry", 0, WDT_TIMEOUT_TELEM_MS,   true, 0, NULL, STACK_SIZE_TELEM * 4},
    {"Command",   0, WDT_TIMEOUT_CMD_MS,     true, 0, NULL, STACK_SIZE_CMD * 4},
    {"Power",     0, WDT_TIMEOUT_POWER_MS,   true, 0, NULL, STACK_SIZE_POWER * 4},
};

// Watchdog ready flag
static volatile bool watchdog_ready = false;

// Task handle
static TaskHandle_t wdt_task_handle = NULL;

// =============================================================================
// Public Functions
// =============================================================================

void watchdog_init() {
    uint32_t now = millis();
    for (int i = 0; i < 4; i++) {
        task_health[i].last_heartbeat_ms = now;
        task_health[i].failure_count = 0;
        task_health[i].handle = NULL;
    }
    watchdog_ready = false;  // Will be set true when task starts
}

void watchdog_register_task(TaskID task_id, TaskHandle_t handle) {
    int idx = (int)task_id;
    if (idx >= 0 && idx < 4) {
        task_health[idx].handle = handle;
    }
}

void watchdog_heartbeat(TaskID task_id) {
    if (!watchdog_ready) return;  // Safety check
    
    int idx = (int)task_id;
    if (idx >= 0 && idx < 4) {
        task_health[idx].last_heartbeat_ms = millis();
    }
}

bool watchdog_is_ready() {
    return watchdog_ready;
}

void watchdog_enable_task(TaskID task_id, bool enabled) {
    int idx = (int)task_id;
    if (idx >= 0 && idx < 4) {
        task_health[idx].enabled = enabled;
    }
}

TaskStats watchdog_get_stats(TaskID task_id) {
    TaskStats stats = {nullptr, 0, 0, false, false};
    
    int idx = (int)task_id;
    if (idx < 0 || idx >= 4) {
        return stats;
    }
    
    uint32_t now = millis();
    uint32_t elapsed = now - task_health[idx].last_heartbeat_ms;
    
    stats.name = task_health[idx].name;
    stats.last_seen_ms_ago = elapsed;
    stats.failure_count = task_health[idx].failure_count;
    stats.enabled = task_health[idx].enabled;
    stats.alive = (elapsed < task_health[idx].timeout_ms);
    
    return stats;
}

// =============================================================================
// Status Reporting Functions
// =============================================================================

void watchdog_print_status() {
    // No header - each line is self-contained with WDT prefix
    
    for (int i = 0; i < 4; i++) {
        TaskStats stats = watchdog_get_stats((TaskID)i);
        
        char msg[80];
        snprintf(msg, sizeof(msg),
                 "HEALTH:%s,%s,%lu,%lu",
                 stats.name,
                 stats.alive ? "ALIVE" : "DEAD",
                 stats.last_seen_ms_ago,
                 stats.failure_count);
        out_system("WDT", msg);
    }
}

void watchdog_print_stack_usage() {
    // No header - each line is self-contained with WDT prefix
    
    for (int i = 0; i < 4; i++) {
        if (task_health[i].handle == NULL) {
            char msg[64];
            snprintf(msg, sizeof(msg), "STACK:%s,NOT_REGISTERED,0,0,0", 
                     task_health[i].name);
            out_system("WDT", msg);
            continue;
        }
        
        // Get high water mark (minimum free stack ever reached)
        UBaseType_t free_stack_words = uxTaskGetStackHighWaterMark(task_health[i].handle);
        uint32_t free_bytes = free_stack_words * 4;
        uint32_t used_bytes = task_health[i].stack_size - free_bytes;
        uint32_t percent_used = (used_bytes * 100) / task_health[i].stack_size;
        
        char msg[96];
        snprintf(msg, sizeof(msg),
                 "STACK:%s,%lu,%lu,%lu,%lu",
                 task_health[i].name,
                 used_bytes,
                 task_health[i].stack_size,
                 percent_used,
                 free_bytes);
        out_system("WDT", msg);
        
        // Warning if stack usage is high
        if (percent_used > 80) {
            char warn[64];
            snprintf(warn, sizeof(warn), "STACK_WARNING:%s,%lu", 
                     task_health[i].name, percent_used);
            out_error("WDT", warn);
        }
    }
    
    // Also check watchdog task itself
    if (wdt_task_handle != NULL) {
        UBaseType_t free_stack_words = uxTaskGetStackHighWaterMark(wdt_task_handle);
        uint32_t free_bytes = free_stack_words * 4;
        uint32_t stack_size = STACK_SIZE_WDT * 4;
        uint32_t used_bytes = stack_size - free_bytes;
        uint32_t percent_used = (used_bytes * 100) / stack_size;
        
        char msg[96];
        snprintf(msg, sizeof(msg),
                 "STACK:%s,%lu,%lu,%lu,%lu",
                 "WDT",
                 used_bytes,
                 stack_size,
                 percent_used,
                 free_bytes);
        out_system("WDT", msg);
        
        if (percent_used > 80) {
            char warn[64];
            snprintf(warn, sizeof(warn), "STACK_WARNING:%s,%lu", "WDT", percent_used);
            out_error("WDT", warn);
        }
    }
}
// =============================================================================
// Watchdog Task (Internal)
// =============================================================================

static void wdt_task(void* param) {
    (void)param;
    
    // Subscribe to hardware watchdog
    esp_task_wdt_add(NULL);
    
    out_system("WDT", "Starting");
    
    // Initialize heartbeats
    uint32_t now = millis();
    for (int i = 0; i < 4; i++) {
        task_health[i].last_heartbeat_ms = now;
        task_health[i].failure_count = 0;
    }
    
    // Signal ready - tasks can now send heartbeats
    watchdog_ready = true;
    
    // Grace period for system stabilization
    vTaskDelay(pdMS_TO_TICKS(2000));
    out_system("WDT", "Monitoring active");
    
    uint32_t check_counter = 0;
    
    while (true) {
        bool all_healthy = true;
        uint32_t current_time = millis();
        
        // Check each task's health
        for (int i = 0; i < 4; i++) {
            if (!task_health[i].enabled) continue;
            
            uint32_t elapsed = current_time - task_health[i].last_heartbeat_ms;
            
            if (elapsed > task_health[i].timeout_ms) {
                all_healthy = false;
                task_health[i].failure_count++;
                
                // Log error with smaller buffer
                char msg[64];
                snprintf(msg, sizeof(msg), 
                         "%s timeout (%lu ms)",
                         task_health[i].name, elapsed);
                out_error("WDT", msg);
            }
        }
        
        if (all_healthy) {
            // Feed hardware watchdog
            esp_task_wdt_reset();
            
            // Periodic health report (every 60 seconds)
            if (check_counter % 60 == 0 && check_counter > 0) {
                out_system("WDT", "All tasks healthy");
            }
        } else {
            // System unhealthy
            if (WDT_ACTION_REBOOT) {
                out_error("WDT", "CRITICAL: Task failure");
                
                // Print task status
                for (int i = 0; i < 4; i++) {
                    uint32_t elapsed = current_time - task_health[i].last_heartbeat_ms;
                    char msg[64];
                    snprintf(msg, sizeof(msg), 
                             "%s: %s (%lu ms ago)",
                             task_health[i].name,
                             (elapsed < task_health[i].timeout_ms) ? "OK" : "DEAD",
                             elapsed);
                    out_error("TASK", msg);
                }
                
                out_system("WDT", "Rebooting in 10s");
                Serial.flush();
                
                // Don't feed watchdog - let hardware watchdog trigger reset
                while (true) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            } else if (WDT_ACTION_LOG_ONLY) {
                // Debug mode - just log, don't reboot
                esp_task_wdt_reset();
            }
        }
        
        check_counter++;
        vTaskDelay(pdMS_TO_TICKS(WDT_CHECK_INTERVAL_MS));
    }
}

TaskHandle_t watchdog_start_task() {
    BaseType_t result = xTaskCreatePinnedToCore(
        wdt_task,
        "WDT",
        STACK_SIZE_WDT,
        NULL,
        PRIORITY_WDT,
        &wdt_task_handle,
        0  // Core 0
    );
    
    if (result != pdPASS) {
        out_error("WDT", "Failed to create task");
        return NULL;
    }
    
    return wdt_task_handle;
}