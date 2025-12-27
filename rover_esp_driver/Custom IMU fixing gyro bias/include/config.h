#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <ArduinoJson.h>

// =========================
// Devices
//============================
extern JsonDocument jsonCmdReceive;
extern JsonDocument jsonInfoSend;
extern JsonDocument jsonInfoHttp;
// =============================================================================
// Hardware Configuration
// =============================================================================

#define I2C_SDA 32
#define I2C_SCL 33
#define I2C_FREQ 400000

#define SERIAL_BAUD 115200

// =============================================================================
// Web Sockets
// =============================================================================

#define WS_RECONNECT_INTERVAL_MS 3000
#define WIFI_CONNECT_TIMEOUT_MS 15000

// =============================================================================
// I2C Addresses
// =============================================================================

#define QMI8658C_ADDR 0x6B
#define AK09918C_ADDR 0x0C

// =============================================================================
// Sensor Configuration
// =============================================================================

// Accelerometer: 0=±2g, 1=±4g, 2=±8g, 3=±16g
#define ACCEL_FS_SEL 3

// Gyroscope: 0=±16dps, 1=±32dps, 2=±64dps, 3=±128dps
//            4=±256dps, 5=±512dps, 6=±1024dps, 7=±2048dps
#define GYRO_FS_SEL 7

// ODR: 0=8000Hz, 1=4000Hz, 2=2000Hz, 3=1000Hz
//      4=500Hz, 5=250Hz, 6=117.5Hz, 7=58.75Hz
#define ODR_SEL 3

#define MAG_SCALE 0.15f  // uT/LSB

// =============================================================================
// Output Configuration
// =============================================================================

#define OUTPUT_RATE_HZ 50
#define OUTPUT_INTERVAL_MS (1000 / OUTPUT_RATE_HZ)

// =============================================================================
// Filter Configuration
// =============================================================================

#define MADGWICK_BETA 0.1f

// Adaptive beta settings (motion-based gain adjustment)
#define ADAPTIVE_BETA_ENABLED     true
#define ADAPTIVE_BETA_STATIONARY  0.5f    // When device is still
#define ADAPTIVE_BETA_MOTION      0.05f   // When device is moving
#define MOTION_THRESHOLD_RADS     0.05f   // ~3 deg/s

// Fast convergence settings
#define FAST_CONVERGENCE_DURATION_MS  2000
#define FAST_CONVERGENCE_BETA         2.5f

// =============================================================================
// JSON Command Types (must match json_cmd.h)
// =============================================================================

// Motor commands (existing - from json_cmd.h)
#define CMD_SPEED_CTRL      1
#define CMD_PWM_INPUT       11
#define CMD_SET_MOTOR_PID   2

// OLED commands (existing)
#define CMD_OLED_CTRL       3
#define CMD_OLED_DEFAULT    -3

// System commands (existing)
#define CMD_REBOOT          600
#define CMD_MM_TYPE_SET     900

// Not sure what they are, investigate
#define FEEDBACK_IMU_DATA   1002

// MODULE TYPE
// 0: nothing
// 1: RoArm-M2-S
// 2: Gimbal
// {"T":4,"cmd":0}
#define CMD_MODULE_TYPE	4

// =============================================================================
// Motor Control
// =============================================================================

// CHANGE HEART BEAT DELAY
// {"T":136,"cmd":3000}
#define CMD_HEART_BEAT_SET	136

// SET SPEED RATE
// {"T":138,"L":1,"R":1}
#define CMD_SET_SPD_RATE	138

// GET SPEED RATE
// {"T":139}
#define CMD_GET_SPD_RATE	139

// SAVE SPEED RATE
// {"T":140}
#define CMD_SAVE_SPD_RATE	140

// =============================================================================
// IMU Control Commands (330-360 range)
// =============================================================================

// IMU Stream commands (existing - from json_cmd.h)
#define CMD_IMU_STREAM_CTRL 325   // {"T":325,"cmd":1}
#define CMD_STREAM_FORMAT   400   // {"T":400,"cmd":1}
// 

// Calibration commands
#define CMD_IMU_CALIBRATE_GYRO   330   // {"T":330}
#define CMD_IMU_CALIBRATE_ACCEL  331   // {"T":331}
#define CMD_IMU_CALIBRATE_ALL    332   // {"T":332}

// Filter commands
#define CMD_IMU_FILTER_RESET     335   // {"T":335}
#define CMD_IMU_SET_BETA         336   // {"T":336,"beta":0.1}

// Magnetometer axis correction
#define CMD_IMU_SET_MAG_SIGNS    340   // {"T":340,"x":1,"y":1,"z":-1}

// Debug/status commands
#define CMD_IMU_DEBUG            345   // {"T":345}
#define CMD_IMU_GET_STATUS       346   // {"T":346}

// Orientation query
#define CMD_IMU_GET_ORIENTATION  350   // {"T":350} -> {"T":350,"yaw":x,"pitch":y,"roll":z}

// =============================================================================
// NEW: Fast Initialization & Adaptive Beta Commands (351-360)
// =============================================================================

// Initialize filter from current sensor readings (instant alignment!)
// This bypasses the slow convergence entirely
#define CMD_IMU_INIT_FROM_SENSORS  351   // {"T":351}

// Start fast convergence mode (high beta for quick alignment)
// duration: milliseconds (0 = until stable)
// beta: fast mode beta (default 2.5)
#define CMD_IMU_FAST_CONVERGENCE   352   // {"T":352,"duration":2000,"beta":2.5}

// Configure adaptive beta (motion-based gain adjustment)
// enabled: 0/1
// stationary: beta when still (default 0.5)
// motion: beta when moving (default 0.05)
// threshold: motion threshold in rad/s (default 0.05)
#define CMD_IMU_ADAPTIVE_BETA      353   // {"T":353,"enabled":1,"stationary":0.5,"motion":0.05,"threshold":0.05}

// Get filter convergence status
#define CMD_IMU_CONVERGENCE_STATUS 354   // {"T":354} -> {"T":354,"rate":0.01,"converged":true,"active_beta":0.1}

// =============================================================================
// WiFi and Websockets
// =============================================================================

// WIFI AP settings (note: lower case 'ssid' and 'pass')
// {"T": 500, "ssid": "abc", "pass": "password"}
#define CMD_WIFI                500

// GET STATUS OF WEBSOCKETS
// {"T": 502}
#define WS_STATUS               501

// START STREAMING WEBSOCKETS
// {"T": 503}
#define WS_START                502

// STOP STREAMING WEBSOCKETS
// {"T": 504}
#define WS_STOP                 503

// =============================================================================
// System replies
// =============================================================================

// IMU stream data response type
// {"T":326,"ax":...,"ay":...,...}
#define FEEDBACK_IMU_STREAM    326

#define FEEDBACK_BASE_INFO  1001

// =============================================================================
// Motor Driver Pins
// =============================================================================

#define PWMA    25      // Motor A PWM
#define AIN2    17      // Motor A input 2
#define AIN1    21      // Motor A input 1
#define BIN1    22      // Motor B input 1
#define BIN2    23      // Motor B input 2
#define PWMB    26      // Motor B PWM

// =============================================================================
// UGV Base Parameters
// =============================================================================

#define THRESHOLD_PWM   23

// =============================================================================
// Heartbeat / Safety
// =============================================================================

#define SERVO_STOP_DELAY    3

// =============================================================================
// Compile-time Constants (safe in headers)
// =============================================================================

constexpr uint16_t ANALOG_WRITE_BITS = 8;
constexpr uint16_t MAX_PWM = (1 << ANALOG_WRITE_BITS) - 1;  // 255
constexpr uint16_t MIN_PWM = MAX_PWM / 4;                    // 63

// =============================================================================
// Global Variables (extern declarations - defined in config.cpp)
// =============================================================================

// Debug & Info Settings
// 0: no debug output, 1: print debug info (default), 2: flow feedback
extern byte InfoPrint;

// Platform Type Configuration
// 1: WAVE ROVER, 2: UGV02 (UGV Rover), 3: UGV01 (UGV Beast)
extern byte mainType;

// 0: Base (no arm/gimbal), 1: RoArm-M2, 2: Gimbal
extern byte moduleType;

// Run new JSON command flag
extern bool runNewJsonCmd;

// PWM channel configuration
extern int freq;
extern int channel_A;
extern int channel_B;

// Motor PID Configuration
extern float __kp;
extern float __ki;
extern float __kd;
extern float windup_limits;

// UGV Base Parameters (defaults for WAVE ROVER)
extern double WHEEL_D;
extern int ONE_CIRCLE_PLUSES;
extern double TRACK_WIDTH;
extern bool SET_MOTOR_DIR;

// Communication Settings
extern int feedbackFlowExtraDelay;
extern bool uartCmdEcho;

// Heartbeat / Safety
extern int HEART_BEAT_DELAY;
extern unsigned long lastCmdRecvTime;

// Timing
extern unsigned long prev_time;

// UART config
extern bool imu_stream_enabled;
extern bool stream_as_json;


// -----------------------------------------------------------------------------
// IMU State
// -----------------------------------------------------------------------------


extern uint32_t last_update_us;
extern uint32_t last_update_ina;



#endif // CONFIG_H
