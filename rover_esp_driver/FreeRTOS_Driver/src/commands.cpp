/*
 * Command Parser Module Implementation
 */

#include "commands.h"
#include "protocol.h"
#include "motors.h"
#include "sensors.h"
#include "madgwick.h"
#include "oled.h"

// Command buffer
static char cmd_buffer[128];
static uint8_t cmd_index = 0;

// =============================================================================
// Initialization
// =============================================================================

void commands_init() {
    cmd_index = 0;
    memset(cmd_buffer, 0, sizeof(cmd_buffer));
}

// =============================================================================
// Command Parsing Helpers
// =============================================================================

// Check if string starts with prefix
static bool starts_with(const char* str, const char* prefix) {
    return strncmp(str, prefix, strlen(prefix)) == 0;
}

// Parse two integers from "a,b" format
static bool parse_two_ints(const char* str, int* a, int* b) {
    char* endptr;
    *a = strtol(str, &endptr, 10);
    if (*endptr != ',') return false;
    *b = strtol(endptr + 1, &endptr, 10);
    return (*endptr == '\0' || *endptr == '\n' || *endptr == '\r');
}

// Parse a float value
static bool parse_float(const char* str, float* val) {
    char* endptr;
    *val = strtof(str, &endptr);
    return (endptr != str);
}

// Parse an integer value
static bool parse_int(const char* str, int* val) {
    char* endptr;
    *val = strtol(str, &endptr, 10);
    return (endptr != str);
}

// =============================================================================
// Command Execution
// =============================================================================

bool commands_execute(const char* cmd) {
    // Trim leading whitespace
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    
    // Empty command
    if (*cmd == '\0' || *cmd == '\n' || *cmd == '\r') {
        return true;
    }
    
    // Motor control: M:left,right
    if (starts_with(cmd, "M:")) {
        int left, right;
        if (parse_two_ints(cmd + 2, &left, &right)) {
            motors_set(left, right);
            out_ack("M");
            return true;
        }
        out_error("M", "invalid format");
        return false;
    }
    
    // Stop motors
    if (starts_with(cmd, "STOP")) {
        motors_stop();
        out_ack("STOP");
        return true;
    }
    
    // Emergency stop
    if (starts_with(cmd, "ESTOP")) {
        motors_emergency_stop();
        out_ack("ESTOP");
        return true;
    }
    
    // Enable motors
    if (starts_with(cmd, "ENABLE")) {
        motors_enable(true);
        out_ack("ENABLE");
        return true;
    }
    
    // Heartbeat
    if (starts_with(cmd, "HB")) {
        motors_heartbeat();
        out_ack("HB");
        return true;
    }
    
    // Stream control: STREAM:ON/OFF
    if (starts_with(cmd, "STREAM:")) {
        const char* arg = cmd + 7;
        if (starts_with(arg, "ON")) {
            protocol_set_streaming(true);
            out_ack("STREAM", "ON");
        } else if (starts_with(arg, "OFF")) {
            protocol_set_streaming(false);
            out_ack("STREAM", "OFF");
        } else {
            out_error("STREAM", "invalid arg");
            return false;
        }
        return true;
    }
    
    // Format control: FMT:RAW/IMU
    if (starts_with(cmd, "FMT:")) {
        const char* arg = cmd + 4;
        if (starts_with(arg, "RAW")) {
            protocol_set_format(StreamFormat::MOTIONCAL);
            out_ack("FMT", "RAW");
        } else if (starts_with(arg, "IMU")) {
            protocol_set_format(StreamFormat::TELEMETRY);
            out_ack("FMT", "IMU");
        } else {
            out_error("FMT", "invalid arg");
            return false;
        }
        return true;
    }
    
    // Initialize filter from sensors
    if (starts_with(cmd, "INIT")) {
        imu.read();
        mag.read();
        if (sensors_mag_ok()) {
            filter.init_from_sensors(
                imu.data().accel[0], imu.data().accel[1], imu.data().accel[2],
                mag.data().mag[0], mag.data().mag[1], mag.data().mag[2]
            );
        } else {
            filter.init_from_accel(
                imu.data().accel[0], imu.data().accel[1], imu.data().accel[2]
            );
        }
        out_ack("INIT");
        return true;
    }
    
    // Status
    if (starts_with(cmd, "STATUS")) {
        out_status(
            sensors_imu_ok(),
            sensors_mag_ok(),
            sensors_pwr_ok(),
            filter.yaw(),
            filter.pitch(),
            filter.roll(),
            pwr.data().load_voltage_V,
            pwr.data().current_mA,
            motors_get_left(),
            motors_get_right()
        );
        return true;
    }
    
    // Reboot
    if (starts_with(cmd, "REBOOT")) {
        out_ack("REBOOT");
        delay(100);
        esp_restart();
        return true;
    }
    
    // Unknown command
    out_error("UNKNOWN", cmd);
    return false;
}

// =============================================================================
// Serial Processing
// =============================================================================

void commands_process() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (cmd_index > 0) {
                cmd_buffer[cmd_index] = '\0';
                commands_execute(cmd_buffer);
                cmd_index = 0;
            }
        } else if (cmd_index < sizeof(cmd_buffer) - 1) {
            cmd_buffer[cmd_index++] = c;
        }
    }
}