/*
 * Rover Driver - Main Entry Point
 * 
 * FreeRTOS-based driver for Waveshare rover with:
 *   - QMI8658C IMU (accelerometer + gyroscope)
 *   - AK09918C magnetometer
 *   - INA219 power monitor
 *   - Dual H-bridge motor control
 * 
 * Tasks:
 *   - IMU Task: Reads sensors, Fuse using Madgwick
 *   - Telemetry Task: Sends telemetry at configured rate
 *   - Command Task: Parses and executes serial commands
 *   - Power Task: Reads power monitor
 *   - Watchdog Task: Monitors system health
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include "config.h"
#include "protocol.h"
#include "sensors.h"
#include "motors.h"
#include "commands.h"
#include "oled.h"
#include "SensorFusion.h"
#include "hardcoded_calibration.h"


// =============================================================================
// Task Health Monitoring
// =============================================================================

enum class TaskID : int {
    IMU = 0,
    TELEMETRY = 1,
    COMMAND = 2,
    POWER = 3,
    TASK_COUNT = 4  // Keep this last - gives us the number of tasks
};

struct TaskHealth {
    const char* name;           // Task name for error messages
    volatile uint32_t last_heartbeat_ms; // Last time task reported in
    uint32_t timeout_ms;        // How long before we consider it dead
    bool enabled;               // Is this task critical?
    volatile uint32_t failure_count;     // How many times has it timed out?
};

// Task health tracking array
static TaskHealth task_health[4] = {
    // name,      last_heartbeat, timeout,                enabled, failures
    {"IMU",       0,              WDT_TIMEOUT_IMU_MS,     true,    0},
    {"Telemetry", 0,              WDT_TIMEOUT_TELEM_MS,   true,    0},
    {"Command",   0,              WDT_TIMEOUT_CMD_MS,     true,    0},
    {"Power",     0,              WDT_TIMEOUT_POWER_MS,   true,    0},
};

// Watchdog ready flag - prevents tasks from calling heartbeat before WDT is ready
static volatile bool watchdog_ready = false;

// Helper function to update task heartbeat
inline void task_heartbeat(TaskID task_id) {
    if (!watchdog_ready) return;  // Safety check
    
    int idx = (int)task_id;
    if (idx >= 0 && idx < 4) {
        task_health[idx].last_heartbeat_ms = millis();
    }
}

// Helper to check if a task has timed out
inline bool task_is_alive(TaskID task_id) {
    int idx = (int)task_id;
    if (idx < 0 || idx >= 4) return false;
    if (!task_health[idx].enabled) return true;
    
    uint32_t now = millis();
    uint32_t elapsed = now - task_health[idx].last_heartbeat_ms;
    return elapsed < task_health[idx].timeout_ms;
}

// =============================================================================
// Sensor Fusion
// =============================================================================

SF fusion;
float deltat;

// =============================================================================
// Task Handles
// =============================================================================

static TaskHandle_t imu_task_handle = NULL;
static TaskHandle_t telem_task_handle = NULL;
static TaskHandle_t cmd_task_handle = NULL;
static TaskHandle_t power_task_handle = NULL;
static TaskHandle_t wdt_task_handle = NULL;

// =============================================================================
// Shared State
// =============================================================================

// static uint32_t last_imu_update_us = 0;
static float cpu_temp = 0;

// Mutex for sensor data access
static SemaphoreHandle_t sensor_mutex = NULL;

// =============================================================================
// IMU Task
// =============================================================================
// Highest priority - reads sensors and updates filter at 100Hz

static void imu_task(void* param) {
    (void)param;
    
    TickType_t last_wake = xTaskGetTickCount();

    while (!watchdog_ready) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Initialize heartbeat
    task_heartbeat(TaskID::IMU);
    
    while (true) {
        // Read sensors
        if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            sensors_read_imu();
                        
            // Update filter
            const IMU_Data& imu_data = imu.data();
            const MAG_Data& mag_data = mag.data();

            deltat = fusion.deltatUpdate(); 
            fusion.MadgwickUpdate(
                    imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2],
                    imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
                    mag_data.mag[0], mag_data.mag[1], mag_data.mag[2],
                    deltat
            );
            task_heartbeat(TaskID::IMU);

            xSemaphoreGive(sensor_mutex);
        }
        
        // Run at 100Hz
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_IMU_MS));
    }
}

// =============================================================================
// Telemetry Task
// =============================================================================
// Sends telemetry data at configured rate

static void telem_task(void* param) {
    (void)param;
    
    TickType_t last_wake = xTaskGetTickCount();

    while (!watchdog_ready) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Initialize heartbeat
    task_heartbeat(TaskID::TELEMETRY);
    
    while (true) {
        if (protocol_get_streaming()) {
            if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                const IMU_Data& imu_data = imu.data();
                const MAG_Data& mag_data = mag.data();
                
                // Get CPU temperature
                cpu_temp = temperatureRead();
                
                if (protocol_get_format() == StreamFormat::MOTIONCAL) {
                    // Send raw magnetometer data for calibration
                    float mag_raw_ut[3] = {
                        mag_data.mag_raw[0] * MAG_SCALE,
                        mag_data.mag_raw[1] * MAG_SCALE,
                        mag_data.mag_raw[2] * MAG_SCALE
                    };
                    
                    out_motioncal_raw(
                        imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
                        imu_data.gyro_dps[0], imu_data.gyro_dps[1], imu_data.gyro_dps[2],
                        mag_raw_ut[0], mag_raw_ut[1], mag_raw_ut[2]
                    );
                    
                    out_motioncal_ori(fusion.getYaw(),
                                      fusion.getPitch(),
                                      fusion.getRoll());
                } else {
                    // Send telemetry format
                    out_imu_telemetry(
                        fusion.getYaw(), fusion.getPitch(), fusion.getRoll(), cpu_temp,
                        imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
                        imu_data.gyro_dps[0], imu_data.gyro_dps[1], imu_data.gyro_dps[2],
                        mag_data.mag[0], mag_data.mag[1], mag_data.mag[2]
                    );
                }
                
                xSemaphoreGive(sensor_mutex);
            }
        }
        
        task_heartbeat(TaskID::TELEMETRY);

        // Run at telemetry rate (e.g., 50Hz)
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEM_INTERVAL_IMU_MS));
    }
}

// =============================================================================
// Command Task
// =============================================================================
// Handles serial command parsing

static void cmd_task(void* param) {
    (void)param;
    
    while (!watchdog_ready) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Initialize heartbeat
    task_heartbeat(TaskID::COMMAND);

    while (true) {
        // Process serial commands
        commands_process();
        
        // Report heartbeat
        task_heartbeat(TaskID::COMMAND);

        // Short delay to prevent busy-waiting
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// =============================================================================
// Power Monitor Task
// =============================================================================
// Reads INA219 at lower rate

static void power_task(void* param) {
    (void)param;
    
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t telem_counter = 0;
    
    while (!watchdog_ready) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Initialize heartbeat
    task_heartbeat(TaskID::POWER);

    while (true) {
        // Read power monitor
        pwr.read();
        
        // Update OLED with power data
        const PWR_Data& data = pwr.data();
        oled_set_power(data.load_voltage_V, data.current_mA, data.power_mW);
        
        // Check motor timeout
        motors_check_timeout();
        
        // Send power telemetry at lower rate
        telem_counter++;
        if (protocol_get_streaming() && (telem_counter >= (TELEM_INTERVAL_POWER_MS / PERIOD_POWER_MS))) {
            telem_counter = 0;
            out_power(data.load_voltage_V, data.current_mA, data.power_mW, data.shunt_voltage_mV);
        }
        
        // Report heartbeat
        task_heartbeat(TaskID::POWER);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_POWER_MS));
    }
}

// =============================================================================
// Watchdog Task - Enhanced with Task Health Monitoring
// =============================================================================

static void wdt_task(void* param) {
    (void)param;
    
    // Subscribe to hardware watchdog
    esp_task_wdt_add(NULL);
    
    out_system("WDT", "Starting");
    
    // Initialize all heartbeats to current time
    uint32_t now = millis();
    for (int i = 0; i < 4; i++) {
        task_health[i].last_heartbeat_ms = now;
        task_health[i].failure_count = 0;
    }
    
    // Signal that watchdog is ready - tasks can now send heartbeats
    watchdog_ready = true;
    
    // Wait for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));
    out_system("WDT", "Monitoring active");
    
    uint32_t check_counter = 0;
    
    while (true) {
        bool all_healthy = true;
        uint32_t current_time = millis();
        
        // Print Stack usage every 2 minutes
            if (check_counter % 300 == 0 && check_counter > 0) {
            out_system("WDT", "=== Stack Usage ===");
            
            if (imu_task_handle) {
                UBaseType_t stack = uxTaskGetStackHighWaterMark(imu_task_handle);
                out_system("  IMU", (int)(stack * 4));  // Convert words to bytes
            }
            if (telem_task_handle) {
                UBaseType_t stack = uxTaskGetStackHighWaterMark(telem_task_handle);
                out_system("  Telem", (int)(stack * 4));
            }
            if (cmd_task_handle) {
                UBaseType_t stack = uxTaskGetStackHighWaterMark(cmd_task_handle);
                out_system("  Cmd", (int)(stack * 4));
            }
            if (power_task_handle) {
                UBaseType_t stack = uxTaskGetStackHighWaterMark(power_task_handle);
                out_system("  Power", (int)(stack * 4));
            }
            if (wdt_task_handle) {
                UBaseType_t stack = uxTaskGetStackHighWaterMark(wdt_task_handle);
                out_system("  WDT", (int)(stack * 4));
            }
        }

        // Check each task
        for (int i = 0; i < 4; i++) {
            if (!task_health[i].enabled) continue;
            
            uint32_t elapsed = current_time - task_health[i].last_heartbeat_ms;
            
            if (elapsed > task_health[i].timeout_ms) {
                all_healthy = false;
                task_health[i].failure_count++;
                
                // Log error
                char msg[80];
                snprintf(msg, sizeof(msg), 
                         "%s timeout (%lu ms, failures: %lu)",
                         task_health[i].name, elapsed, 
                         task_health[i].failure_count);
                out_error("WDT", msg);
            }
        }
        
        if (all_healthy) {
            // Feed hardware watchdog
            esp_task_wdt_reset();
            
            // Periodic status (every 60 seconds)
            if (check_counter % 60 == 0 && check_counter > 0) {
                out_system("WDT", "All tasks healthy");
            }
        } else {
            // System unhealthy
            if (WDT_ACTION_REBOOT) {
                out_error("WDT", "CRITICAL: Task failure");
                out_system("WDT", "Dumping task status");
                
                // Print all task status
                for (int i = 0; i < 4; i++) {
                    uint32_t elapsed = current_time - task_health[i].last_heartbeat_ms;
                    char status[100];
                    snprintf(status, sizeof(status),
                             "%s: %s (last: %lu ms ago, fails: %lu)",
                             task_health[i].name,
                             (elapsed < task_health[i].timeout_ms) ? "OK" : "DEAD",
                             elapsed, task_health[i].failure_count);
                    out_system("  ", status);
                }
                
                out_system("WDT", "System will reboot in 10s");
                Serial.flush();
                
                // Don't feed watchdog - let it trigger reset
                while (true) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            } else if (WDT_ACTION_LOG_ONLY) {
                // Debug mode - just log
                esp_task_wdt_reset();
            }
        }
        
        check_counter++;
        vTaskDelay(pdMS_TO_TICKS(WDT_CHECK_INTERVAL_MS));
    }
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
    // Initialize protocol first (Serial)
    protocol_init();
    out_system("BOOT", "starting");

    // Retrieve configuration
    preferences_init();
    
    // Create mutex for sensor data
    sensor_mutex = xSemaphoreCreateMutex();
    if (sensor_mutex == NULL) {
        out_error("BOOT", "Failed to create sensor mutex!");
        while(1) { delay(1000); }  // Halt
    }
    
    // Find initial guess for Quaternion to 
    // reduce drift
    sensors_init();
    sensors_read_imu();
    motors_init();
    commands_init();
    oled_init();  // Initialize OLED 
    
    // Configure watchdog
    esp_task_wdt_init(WDT_TIMEOUT_SEC, true);  // true = panic on timeout
    out_system("BOOT", "Creating watchdog task");
    
    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(
        wdt_task,
        "WDT",
        STACK_SIZE_WDT,
        NULL,
        PRIORITY_WDT,
        &wdt_task_handle,
        0
    );

    delay(100);

    out_system("BOOT", "Creating other tasks");
    xTaskCreatePinnedToCore(
        imu_task,
        "IMU",
        STACK_SIZE_IMU,
        NULL,
        PRIORITY_IMU,
        &imu_task_handle,
        1  // Core 1 for time-critical sensor reading
    );
    
    xTaskCreatePinnedToCore(
        telem_task,
        "Telem",
        STACK_SIZE_TELEM,
        NULL,
        PRIORITY_TELEM,
        &telem_task_handle,
        0  // Core 0 for I/O
    );
    
    xTaskCreatePinnedToCore(
        cmd_task,
        "Cmd",
        STACK_SIZE_CMD,
        NULL,
        PRIORITY_CMD,
        &cmd_task_handle,
        0  // Core 0 for I/O
    );
    
    xTaskCreatePinnedToCore(
        power_task,
        "Power",
        STACK_SIZE_POWER,
        NULL,
        PRIORITY_POWER,
        &power_task_handle,
        0
    );
    

    
    out_system("BOOT", "ready");
}

// =============================================================================
// Loop
// =============================================================================
// Not used - all work is done in FreeRTOS tasks

void loop() {
    // Yield to FreeRTOS
    vTaskDelete(NULL);
}