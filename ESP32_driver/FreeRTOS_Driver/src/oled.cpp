/*
 * OLED Display Module Implementation
 * 
 * Simple static display showing battery voltage, current, and percentage.
 */

#include "oled.h"
#include "protocol.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// =============================================================================
// Configurable refresh interval (adjust as needed)
// =============================================================================

uint32_t oled_refresh_interval_ms = 60000;  // 1 minute default

// =============================================================================
// Battery percentage calculation
// =============================================================================

// Adjust these for your battery type
// Default: 3S LiPo (9.0V empty, 12.6V full)
static const float BATTERY_MIN_V = 9.0f;
static const float BATTERY_MAX_V = 12.65f;

static uint8_t calc_battery_percent(float voltage) {
    if (voltage <= BATTERY_MIN_V) return 0;
    if (voltage >= BATTERY_MAX_V) return 100;
    return (uint8_t)(((voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V)) * 100.0f);
}

// =============================================================================
// Display Instance
// =============================================================================

static Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
static bool oled_ok = false;

// =============================================================================
// Display Data (volatile for thread safety)
// =============================================================================

static volatile float disp_voltage = 0.0f;
static volatile float disp_current = 0.0f;
static volatile float disp_power = 0.0f;

// Task handle
static TaskHandle_t oled_task_handle = NULL;

// =============================================================================
// Draw Function
// =============================================================================

static void draw_battery_info(float voltage, float current, float power) {
    uint8_t percent = calc_battery_percent(voltage);
    
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    
    // Line 1: Battery percentage (large)
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(percent);
    display.print("%");
    
    // Draw battery icon on the right
    int batt_x = 80;
    int batt_y = 2;
    int batt_w = 40;
    int batt_h = 14;
    
    // Battery outline
    display.drawRect(batt_x, batt_y, batt_w - 4, batt_h, SSD1306_WHITE);
    display.fillRect(batt_x + batt_w - 4, batt_y + 3, 4, batt_h - 6, SSD1306_WHITE);
    
    // Battery fill based on percentage
    int fill_w = ((batt_w - 8) * percent) / 100;
    if (fill_w > 0) {
        display.fillRect(batt_x + 2, batt_y + 2, fill_w, batt_h - 4, SSD1306_WHITE);
    }
    
    // Line 2: Voltage and Current (small)
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print(voltage, 2);
    display.print("V  ");
    display.print(current, 0);
    display.print("mA");
    
    display.display();
}

// =============================================================================
// OLED Task
// =============================================================================

static void oled_task(void* param) {
    (void)param;
    
    // Show splash (white screen) briefly
    display.fillRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SSD1306_WHITE);
    display.display();
    vTaskDelay(pdMS_TO_TICKS(250));
    
    // Initial draw
    draw_battery_info(disp_voltage, disp_current, disp_power);
    
    while (true) {
        // Wait for refresh interval
        vTaskDelay(pdMS_TO_TICKS(oled_refresh_interval_ms));
        
        // Read volatile data and update display
        float v = disp_voltage;
        float c = disp_current;
        float p = disp_power;
        
        draw_battery_info(v, c, p);
    }
}

// =============================================================================
// Public Functions
// =============================================================================

void oled_init() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        out_error("OLED", "init failed");
        oled_ok = false;
        return;
    }
    
    oled_ok = true;
    display.clearDisplay();
    display.display();
    
    delay(50);
    
    xTaskCreatePinnedToCore(
        oled_task,
        "OLED",
        OLED_TASK_STACK,
        NULL,
        OLED_TASK_PRIORITY,
        &oled_task_handle,
        0
    );
    
    out_system("OLED", "OK");
}

void oled_set_power(float voltage_V, float current_mA, float power_mW) {
    disp_voltage = voltage_V;
    disp_current = current_mA;
    disp_power = power_mW;
}

bool oled_is_ok() {
    return oled_ok;
}