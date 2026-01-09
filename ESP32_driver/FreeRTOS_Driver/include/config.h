/*
 * Rover Driver Configuration
 * 
 * All hardware definitions, pin assignments, and system settings.
 * Single source of truth for configuration.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =============================================================================
// Preferences
// =============================================================================

// String in NVS where Calibration boolean flag is stored
#define CONFIGKEY          "CONFIG"
#define CALIBKEY           "USECALIB"
#define STREAMONBOOT       "STREAMONBOOT" 

// =============================================================================
// Hardware Pin Definitions
// =============================================================================

// I2C Bus
#define PIN_I2C_SDA         32
#define PIN_I2C_SCL         33
#define I2C_FREQ            400000

// Motor Driver (H-Bridge)
#define PIN_MOTOR_A_PWM     25
#define PIN_MOTOR_A_IN1     21
#define PIN_MOTOR_A_IN2     17
#define PIN_MOTOR_B_PWM     26
#define PIN_MOTOR_B_IN1     22
#define PIN_MOTOR_B_IN2     23

// OLED Display (optional)
#define OLED_WIDTH          128
#define OLED_HEIGHT         32
#define OLED_ADDR           0x3C

// =============================================================================
// I2C Device Addresses
// =============================================================================

#define ADDR_QMI8658C       0x6B    // IMU
#define ADDR_AK09918C       0x0C    // Magnetometer
#define ADDR_INA219         0x42    // Power monitor

// =============================================================================
// Sensor Configuration
// =============================================================================

// QMI8658C IMU Settings
// Accelerometer: 0=±2g, 1=±4g, 2=±8g, 3=±16g
#define IMU_ACCEL_FS        3

// Gyroscope: 0=±16dps, 1=±32dps, 2=±64dps, 3=±128dps,
//            4=±256dps, 5=±512dps, 6=±1024dps, 7=±2048dps
#define IMU_GYRO_FS         7

// ODR: 0=8000Hz, 1=4000Hz, 2=2000Hz, 3=1000Hz,
//      4=500Hz, 5=250Hz, 6=117.5Hz, 7=58.75Hz
#define IMU_ODR             0

// AK09918C Magnetometer
#define MAG_SCALE           0.15f   // µT per LSB

// =============================================================================
// Motor Configuration
// =============================================================================

#define MOTOR_PWM_FREQ      20000   // 20kHz for quiet operation
#define MOTOR_PWM_BITS      8       // 8-bit resolution (0-255)
#define MOTOR_PWM_CHANNEL_A 0
#define MOTOR_PWM_CHANNEL_B 1

// Deadband - motors won't respond below this PWM value
#define MOTOR_DEADBAND      20

// Heartbeat timeout - stop motors if no command received (ms)
#define MOTOR_TIMEOUT_MS    3000

// =============================================================================
// Communication Settings
// =============================================================================

#define SERIAL_BAUD         115200

// Telemetry rates (Hz)
#define TELEM_RATE_IMU      100      // IMU data at 50Hz
#define TELEM_RATE_POWER    5       // Power data at 5Hz

// =============================================================================
// FreeRTOS Task Configuration
// =============================================================================

// Stack sizes (words, not bytes - multiply by 4 for bytes)
#define STACK_SIZE_IMU      4096
#define STACK_SIZE_TELEM    3072
#define STACK_SIZE_CMD      3072
#define STACK_SIZE_POWER    3072
#define STACK_SIZE_WDT      3072

// Task priorities (higher = more important)
#define PRIORITY_IMU        5       // Highest - sensor fusion
#define PRIORITY_TELEM      3       // Medium - data output
#define PRIORITY_CMD        4       // High - command handling
#define PRIORITY_POWER      2       // Low - power monitoring
#define PRIORITY_WDT        1       // Lowest - watchdog

// Task periods (ms)
#define PERIOD_IMU_MS       10      // 100Hz sensor read
#define PERIOD_POWER_MS     200     // 5Hz power read
#define PERIOD_WDT_MS       1000    // 1Hz watchdog check

// =============================================================================
// Watchdog Configuration
// =============================================================================

#define WDT_TIMEOUT_SEC     10      // Reset if tasks don't respond for 10s

// Task Health Monitoring
#define WDT_MONITOR_ENABLED     true    // Enable task health monitoring
#define WDT_CHECK_INTERVAL_MS   1000    // How often to check task health

// Task timeout thresholds (ms) - how long before we consider a task dead
#define WDT_TIMEOUT_IMU_MS      500     // IMU should update every 10ms, 500ms = very stuck
#define WDT_TIMEOUT_TELEM_MS    2000    // Telemetry can be slower
#define WDT_TIMEOUT_CMD_MS      5000    // Commands are sporadic, allow more time
#define WDT_TIMEOUT_POWER_MS    1000    // Power monitoring should be regular

// Watchdog actions
#define WDT_ACTION_REBOOT       true    // Reboot on task failure
#define WDT_ACTION_LOG_ONLY     false   // Just log errors (for debugging)

// =============================================================================
// Protocol Message Prefixes
// =============================================================================
// These prefixes allow the Python receiver to route messages appropriately.
// Format: "PREFIX:data,data,data\n"

// Output prefixes (device -> host)
#define MSG_RAW         "Raw"   // MotionCal raw data (keep exact format)
#define MSG_ORI         "Ori"   // MotionCal orientation (keep exact format)
#define MSG_IMU         "I"     // IMU telemetry: yaw,pitch,roll,temp
#define MSG_POWER       "P"     // Power: voltage_mV,current_mA,power_mW
#define MSG_SYSTEM      "S"     // System info/status
#define MSG_ACK         "A"     // Command acknowledgment
#define MSG_ERROR       "E"     // Error message
#define MSG_DEBUG       "D"     // Debug output

// =============================================================================
// Compile-time Calculations
// =============================================================================

constexpr uint16_t MOTOR_PWM_MAX = (1 << MOTOR_PWM_BITS) - 1;  // 255

// Telemetry intervals in ms
constexpr uint32_t TELEM_INTERVAL_IMU_MS = 1000 / TELEM_RATE_IMU;
constexpr uint32_t TELEM_INTERVAL_POWER_MS = 1000 / TELEM_RATE_POWER;

// Magnetometer field strength validation
// #define MAG_MAGNITUDE_WARNING_THRESHOLD  5.0f  // Warn if >5µT deviation

#endif // CONFIG_H
