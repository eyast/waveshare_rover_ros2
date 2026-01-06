#include "config.h"

// =============================================================================
// Global Variable Definitions
// =============================================================================

// Debug & Info Settings
// 0: no debug output, 1: print debug info (default), 2: flow feedback
//byte InfoPrint = 1;

// Platform Type Configuration
// 1: WAVE ROVER, 2: UGV02 (UGV Rover), 3: UGV01 (UGV Beast)
byte mainType = 1;

// 0: Base (no arm/gimbal), 1: RoArm-M2, 2: Gimbal
byte moduleType = 0;

// Run new JSON command flag
//bool runNewJsonCmd = false;

// PWM channel configuration
int freq = 100000;
int channel_A = 5;
int channel_B = 6;

// Motor PID Configuration
float __kp = 20.0f;
float __ki = 2000.0f;
float __kd = 0.0f;
float windup_limits = 255.0f;

// UGV Base Parameters (defaults for WAVE ROVER)
double WHEEL_D = 0.0800;
int ONE_CIRCLE_PLUSES = 2100;
double TRACK_WIDTH = 0.125;
bool SET_MOTOR_DIR = false;

// Communication Settings
int feedbackFlowExtraDelay = 0;
bool uartCmdEcho = false;

// Heartbeat / Safety
int HEART_BEAT_DELAY = 3000;
unsigned long lastCmdRecvTime = 0;  // Will be set to millis() in setup()

// Timing
unsigned long prev_time = 0;

// ========== Devices
//
JsonDocument jsonCmdReceive;
JsonDocument jsonInfoSend;
JsonDocument jsonInfoHttp;


// -----------------------------------------------------------------------------
// IMU State
// -----------------------------------------------------------------------------


uint32_t last_update_us = 0;
uint32_t last_update_ina = 0;
