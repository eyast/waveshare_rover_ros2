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
    
    while (true) {
        // Process serial commands
        commands_process();
        
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
        
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_POWER_MS));
    }
}

// =============================================================================
// Watchdog Task
// =============================================================================
// Monitors system health and feeds watchdog

static void wdt_task(void* param) {
    (void)param;
    
    // Subscribe this task to watchdog
    esp_task_wdt_add(NULL);
    
    while (true) {
        // Feed the watchdog
        esp_task_wdt_reset();
        
        // Could add more health checks here
        
        vTaskDelay(pdMS_TO_TICKS(PERIOD_WDT_MS));
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
    
    // Find initial guess for Quaternion to 
    // reduce drift
    sensors_init();
    sensors_read_imu();
    motors_init();
    commands_init();
    oled_init();  // Initialize OLED 
    
    // Configure watchdog
    esp_task_wdt_init(WDT_TIMEOUT_SEC, true);  // true = panic on timeout
    
    // Create FreeRTOS tasks
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
    
    xTaskCreatePinnedToCore(
        wdt_task,
        "WDT",
        STACK_SIZE_WDT,
        NULL,
        PRIORITY_WDT,
        &wdt_task_handle,
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