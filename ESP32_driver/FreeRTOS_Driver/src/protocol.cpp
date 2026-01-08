/*
 * Protocol Module Implementation
 */

#include "protocol.h"

// =============================================================================
// Protocol State
// =============================================================================

ProtocolState protocol = {
    .output_mode = OutputMode::UART,
    .stream_format = StreamFormat::MOTIONCAL,
    .streaming_enabled = true

};

// Internal buffer for message formatting
static char msg_buffer[256];

// =============================================================================
// Preferences
// =============================================================================

Preferences preferences;
bool CALIB;
bool USE_HARDCODED_CAL;
bool USE_ACCELEROMETER_CAL;
bool USE_GYROSCOPE_CALIB;

void preferences_init() {
    preferences.begin(CONFIGKEY, false);
    if (!preferences.isKey(CALIBKEY)) {
        out_system("NVS", "NVS does not contain USECALILB flag");
        CALIB = false;
        preferences.putBool(CALIBKEY, false); 
    } else {
        CALIB = preferences.getBool(CALIBKEY);
        out_system("NVS", "NVS contains CALIB flag");
        out_system("NVS", CALIB);
  }
  USE_HARDCODED_CAL, USE_ACCELEROMETER_CAL, USE_GYROSCOPE_CALIB = CALIB;
  preferences.end();
}

void calib_set_flag(bool enabled){
    preferences.begin(CONFIGKEY, false);
    preferences.putBool(CALIBKEY, enabled);
    preferences.end();
}

bool calib_get_flag(){
    preferences.begin(CONFIGKEY, true);
    bool retval;
    retval = preferences.getBool(CALIBKEY);
    preferences.end();
    return retval;
}

// =============================================================================
// Protocol Initiliazation
// =============================================================================

void protocol_init() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000) {
        delay(10);
    }
}

// =============================================================================
// Output Mode Control
// =============================================================================

void protocol_set_mode(OutputMode mode) {
    protocol.output_mode = mode;
}

OutputMode protocol_get_mode() {
    return protocol.output_mode;
}

void protocol_set_format(StreamFormat format) {
    protocol.stream_format = format;
}

StreamFormat protocol_get_format() {
    return protocol.stream_format;
}

void protocol_set_streaming(bool enabled) {
    protocol.streaming_enabled = enabled;
}

bool protocol_get_streaming() {
    return protocol.streaming_enabled;
}

// =============================================================================
// Low-Level Output
// =============================================================================

// Send a line to the current output channel
static void send_line(const char* line) {
        Serial.println(line);
}

void out_raw(const char* line) {
    send_line(line);
}

// =============================================================================
// System Messages
// =============================================================================

void out_system(const char* msg) {
    snprintf(msg_buffer, sizeof(msg_buffer), "%s:%s", MSG_SYSTEM, msg);
    send_line(msg_buffer);
}

void out_system(const char* msg, int value) {
    snprintf(msg_buffer, sizeof(msg_buffer), "%s:%s,%d", MSG_SYSTEM, msg, value);
    send_line(msg_buffer);
}

void out_system(const char* msg, float value) {
    snprintf(msg_buffer, sizeof(msg_buffer), "%s:%s,%.2f", MSG_SYSTEM, msg, value);
    send_line(msg_buffer);
}

void out_system(const char* msg, const char* value) {
    snprintf(msg_buffer, sizeof(msg_buffer), "%s:%s,%s", MSG_SYSTEM, msg, value);
    send_line(msg_buffer);
}

// =============================================================================
// Acknowledgments
// =============================================================================

void out_ack(const char* cmd) {
    snprintf(msg_buffer, sizeof(msg_buffer), "%s:%s", MSG_ACK, cmd);
    send_line(msg_buffer);
}

void out_ack(const char* cmd, const char* detail) {
    snprintf(msg_buffer, sizeof(msg_buffer), "%s:%s,%s", MSG_ACK, cmd, detail);
    send_line(msg_buffer);
}

// =============================================================================
// Errors
// =============================================================================

void out_error(const char* msg) {
    snprintf(msg_buffer, sizeof(msg_buffer), "%s:%s", MSG_ERROR, msg);
    send_line(msg_buffer);
}

void out_error(const char* msg, const char* detail) {
    snprintf(msg_buffer, sizeof(msg_buffer), "%s:%s,%s", MSG_ERROR, msg, detail);
    send_line(msg_buffer);
}

// =============================================================================
// Debug
// =============================================================================

void out_debug(const char* msg) {
#ifdef DEBUG
    snprintf(msg_buffer, sizeof(msg_buffer), "%s:%s", MSG_DEBUG, msg);
    send_line(msg_buffer);
#endif
}

// =============================================================================
// MotionCal Output
// =============================================================================

// MotionCal expects specific scaling:
//   Accel: 8192 LSB/g
//   Gyro: 16 LSB/dps
//   Mag: 10 LSB/µT
#define MOTIONCAL_ACCEL_SCALE 8192.0f
#define MOTIONCAL_GYRO_SCALE  16.0f
#define MOTIONCAL_MAG_SCALE   10.0f

void out_motioncal_raw(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float mx, float my, float mz) {
    // Convert to MotionCal's expected "raw" format
    int accel_x = (int)(ax * MOTIONCAL_ACCEL_SCALE);
    int accel_y = (int)(ay * MOTIONCAL_ACCEL_SCALE);
    int accel_z = (int)(az * MOTIONCAL_ACCEL_SCALE);
    
    int gyro_x = (int)(gx * MOTIONCAL_GYRO_SCALE);
    int gyro_y = (int)(gy * MOTIONCAL_GYRO_SCALE);
    int gyro_z = (int)(gz * MOTIONCAL_GYRO_SCALE);
    
    int mag_x = (int)(mx * MOTIONCAL_MAG_SCALE);
    int mag_y = (int)(my * MOTIONCAL_MAG_SCALE);
    int mag_z = (int)(mz * MOTIONCAL_MAG_SCALE);
    // float accel_x = (ax * MOTIONCAL_ACCEL_SCALE);
    // float accel_y = (ay * MOTIONCAL_ACCEL_SCALE);
    // float accel_z = (az * MOTIONCAL_ACCEL_SCALE);
    
    // float gyro_x = (gx * MOTIONCAL_GYRO_SCALE);
    // float gyro_y = (gy * MOTIONCAL_GYRO_SCALE);
    // float gyro_z = (gz * MOTIONCAL_GYRO_SCALE);
    
    // float mag_x = (mx * MOTIONCAL_MAG_SCALE);
    // float mag_y = (my * MOTIONCAL_MAG_SCALE);
    // float mag_z = (mz * MOTIONCAL_MAG_SCALE);
    
    // Exact format expected by MotionCal
    snprintf(msg_buffer, sizeof(msg_buffer),
             "Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d",
             accel_x, accel_y, accel_z,
             gyro_x, gyro_y, gyro_z,
             mag_x, mag_y, mag_z);
    send_line(msg_buffer);
}

void out_motioncal_ori(float yaw, float pitch, float roll) {
    // MotionCal expects "Ori: " (with space after colon)
    snprintf(msg_buffer, sizeof(msg_buffer),
             "Ori: %.2f,%.2f,%.2f",
             yaw, pitch, roll);
    send_line(msg_buffer);
}

// =============================================================================
// IMU Telemetry
// =============================================================================

void out_imu_telemetry(float yaw, float pitch, float roll, float temp,
                       float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float mx, float my, float mz) {
    snprintf(msg_buffer, sizeof(msg_buffer),
             "%s:%.2f,%.2f,%.2f,%.1f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f",
             MSG_IMU,
             yaw, pitch, roll, temp,
             ax, ay, az,
             gx, gy, gz,
             mx, my, mz);
    send_line(msg_buffer);
}

// =============================================================================
// Power Telemetry
// =============================================================================

void out_power(float voltage_V, float current_mA, float power_mW, float shunt_mV) {
    snprintf(msg_buffer, sizeof(msg_buffer),
             "%s:%.3f,%.2f,%.2f,%.3f",
             MSG_POWER,
             voltage_V, current_mA, power_mW, shunt_mV);
    send_line(msg_buffer);
}


