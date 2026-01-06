/*
 * Protocol Module
 * 
 * Abstracts the output channel (Serial or WebSocket) and provides
 * clean message formatting functions with prefixes for Python routing.
 * 
 * Message Format: "PREFIX:data,data,data\n"
 * 
 * The prefix system allows the Python receiver to easily route messages:
 *   - "Raw:" and "Ori:" -> MotionCal application
 *   - "I:" -> IMU telemetry handler
 *   - "P:" -> Power monitor handler
 *   - "S:" -> System log
 *   - "A:" -> Command acknowledgment
 *   - "E:" -> Error handler
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>
#include "config.h"

// =============================================================================
// Output Channel Abstraction
// =============================================================================

// Output mode enumeration
enum class OutputMode {
    UART,       // Default: output to Serial port
    WEBSOCKET   // Route to WebSocket when connected
};

// Stream format for IMU data
enum class StreamFormat {
    MOTIONCAL,  // Raw:/Ori: format for MotionCal calibration tool
    TELEMETRY   // I: format for normal operation
};

// =============================================================================
// Protocol State
// =============================================================================

struct ProtocolState {
    OutputMode output_mode;
    StreamFormat stream_format;
    bool streaming_enabled;
};

extern ProtocolState protocol;

// =============================================================================
// Initialization
// =============================================================================

void protocol_init();

// =============================================================================
// Output Mode Control
// =============================================================================

void protocol_set_mode(OutputMode mode);
OutputMode protocol_get_mode();

void protocol_set_format(StreamFormat format);
StreamFormat protocol_get_format();

void protocol_set_streaming(bool enabled);
bool protocol_get_streaming();


// =============================================================================
// Message Output Functions
// =============================================================================

// Low-level: send a raw line (adds newline automatically)
void out_raw(const char* line);

// System messages
void out_system(const char* msg);
void out_system(const char* msg, int value);
void out_system(const char* msg, float value);
void out_system(const char* msg, const char* value);

// Acknowledgments
void out_ack(const char* cmd);
void out_ack(const char* cmd, const char* detail);

// Errors
void out_error(const char* msg);
void out_error(const char* msg, const char* detail);

// Debug (only sent when DEBUG is enabled)
void out_debug(const char* msg);

// =============================================================================
// Telemetry Output Functions
// =============================================================================

// MotionCal format (exact format required by the tool)
// "Raw:ax,ay,az,gx,gy,gz,mx,my,mz"
void out_motioncal_raw(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float mx, float my, float mz);

// "Ori: yaw,pitch,roll"
void out_motioncal_ori(float yaw, float pitch, float roll);

// IMU telemetry format
// "I:yaw,pitch,roll,temp,ax,ay,az,gx,gy,gz,mx,my,mz"
void out_imu_telemetry(float yaw, float pitch, float roll, float temp,
                       float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float mx, float my, float mz);

// Power telemetry format
// "P:voltage_mV,current_mA,power_mW,shunt_mV"
void out_power(float voltage_V, float current_mA, float power_mW, float shunt_mV);

// =============================================================================
// Status Output
// =============================================================================

// Send complete system status
void out_status(bool imu_ok, bool mag_ok, bool ina_ok,
                float yaw, float pitch, float roll,
                float voltage_V, float current_mA,
                int motor_left, int motor_right);


#endif // PROTOCOL_H
