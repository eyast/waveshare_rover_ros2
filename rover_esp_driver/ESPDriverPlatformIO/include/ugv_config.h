// =============================================================================
// UGV Platform Configuration
// =============================================================================

#ifndef UGV_CONFIG_H
#define UGV_CONFIG_H

// =============================================================================
// Debug & Info Settings
// =============================================================================

// 0: no debug output, 1: print debug info (default), 2: flow feedback
byte InfoPrint = 1;

// =============================================================================
// ESP-NOW Configuration
// =============================================================================

// 0: none, 1: flow-leader(group), 2: flow-leader(single), 3: follower (default)
byte espNowMode = 0;

// Allow control via broadcast MAC address (FF:FF:FF:FF:FF:FF)
bool ctrlByBroadcast = true;

// Broadcast MAC whitelist
uint8_t mac_whitelist_broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

String thisMacStr;

// =============================================================================
// End-Effector Configuration
// =============================================================================

// 0: end servo as grab, 1: end servo as vertical joint
byte EEMode = 0;

// =============================================================================
// Platform Type Configuration
// =============================================================================

// 1: WAVE ROVER, 2: UGV02 (UGV Rover), 3: UGV01 (UGV Beast)
byte mainType = 1;

// 0: Base (no arm/gimbal), 1: RoArm-M2, 2: Gimbal
byte moduleType = 0;

// Run new JSON command flag
bool runNewJsonCmd = false;

// =============================================================================
// Motor Driver Pins
// =============================================================================

const uint16_t ANALOG_WRITE_BITS = 8;
const uint16_t MAX_PWM = pow(2, ANALOG_WRITE_BITS) - 1;
const uint16_t MIN_PWM = MAX_PWM / 4;

#define PWMA    25      // Motor A PWM
#define AIN2    17      // Motor A input 2
#define AIN1    21      // Motor A input 1
#define BIN1    22      // Motor B input 1
#define BIN2    23      // Motor B input 2
#define PWMB    26      // Motor B PWM

#define AENCA   35      // Encoder A channel A
#define AENCB   34      // Encoder A channel B
#define BENCB   16      // Encoder B channel B
#define BENCA   27      // Encoder B channel A

int freq = 100000;
int channel_A = 5;
int channel_B = 6;

// =============================================================================
// I2C Configuration
// =============================================================================

#define S_SCL   33
#define S_SDA   32

// =============================================================================
// Servo UART Pins
// =============================================================================

#define S_RXD   18
#define S_TXD   19

#define RoArmM2_Servo_RXD   18
#define RoArmM2_Servo_TXD   19

#define MAX_SERVO_ID    32

// =============================================================================
// Bus Servo PID Settings
// =============================================================================

#define ST_PID_P_ADDR       21
#define ST_PID_D_ADDR       22
#define ST_PID_I_ADDR       23

#define ST_PID_ROARM_P      16
#define ST_PID_DEFAULT_P    32

#define ST_TORQUE_MAX       1000
#define ST_TORQUE_MIN       50

// =============================================================================
// GPIO / LED Pins
// =============================================================================

#define IO4_PIN     4
#define IO5_PIN     5

int IO4_CH = 7;
int IO5_CH = 8;

const uint16_t FREQ = 200;

// =============================================================================
// Motor PID Configuration
// =============================================================================

float __kp = 20.0;
float __ki = 2000.0;
float __kd = 0;
float windup_limits = 255;

// =============================================================================
// UGV Base Parameters
// =============================================================================

#define THRESHOLD_PWM   23

// Default values (WAVE ROVER)
double WHEEL_D = 0.0800;
int ONE_CIRCLE_PLUSES = 2100;
double TRACK_WIDTH = 0.125;
bool SET_MOTOR_DIR = false;

// =============================================================================
// Communication Settings
// =============================================================================

int feedbackFlowExtraDelay = 0;
bool uartCmdEcho = 1;

// =============================================================================
// Heartbeat / Safety
// =============================================================================

#define SERVO_STOP_DELAY    3

int HEART_BEAT_DELAY = 3000;
unsigned long lastCmdRecvTime = millis();

// =============================================================================
// Timing
// =============================================================================

unsigned long prev_time = 0;

// =============================================================================
// Web / Constant Moving
// =============================================================================

// #define MOVE_STOP       0
// #define MOVE_INCREASE   1
// #define MOVE_DECREASE   2

// #define CONST_ANGLE     0
// #define CONST_XYZT      1

// float const_spd;
// byte const_mode;

// byte const_cmd_base_x;
// byte const_cmd_shoulder_y;
// byte const_cmd_elbow_z;
// byte const_cmd_eoat_t;

// String jsonFeedbackWeb = "";

// =============================================================================
// IMU Data (legacy, for reference)
// =============================================================================

// double icm_pitch, icm_roll, icm_yaw, icm_temp;
// unsigned long last_imu_update = 0;

// double qw, qx, qy, qz;
// double ax, ay, az;
// double mx, my, mz;
// double gx, gy, gz;

// // =============================================================================
// // Magnetometer Calibration (legacy)
// // =============================================================================

// bool IMU_DEBUG_RAW = true;

// int16_t mx_raw = 0, my_raw = 0, mz_raw = 0;
// int16_t mag_offset_x = 37, mag_offset_y = -28, mag_offset_z = -12;

// bool MAG_AUTO_CAL = true;
// int MAG_CAL_MIN_SAMPLES = 100;

// bool mag_cal_valid = false;
// int16_t mag_min[3] = {32767, 32767, 32767};
// int16_t mag_max[3] = {-32768, -32768, -32768};
// int16_t mag_offset_auto[3] = {0, 0, 0};
// int mag_cal_samples = 0;

#endif // UGV_CONFIG_H
