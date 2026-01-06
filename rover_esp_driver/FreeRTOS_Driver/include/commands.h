/*
 * Command Parser Module
 * 
 * Handles parsing and execution of serial commands.
 * 
 * Command Format:
 *   COMMAND:arg1,arg2,...
 * 
 * Available Commands:
 *   M:left,right    - Set motor speeds (-255 to 255)
 *   STOP            - Stop motors
 *   ESTOP           - Emergency stop (disable motors)
 *   ENABLE          - Re-enable motors after ESTOP
 *   
 *   STREAM:ON       - Enable telemetry streaming
 *   STREAM:OFF      - Disable telemetry streaming
 *   FMT:RAW         - Use MotionCal format (Raw:/Ori:)
 *   FMT:IMU         - Use telemetry format (I:)
 *   
 *   INIT            - Re-initialize filter from sensors
 *   
 *   STATUS          - Get system status
 *   REBOOT          - Reboot device
 *   HB              - Heartbeat (reset motor timeout)
 */

#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>

// =============================================================================
// Command Parser
// =============================================================================

// Initialize command system
void commands_init();

// Process incoming serial data
// Call this in the command task loop
void commands_process();

// Parse and execute a command string
// Returns true if command was valid
bool commands_execute(const char* cmd);

#endif // COMMANDS_H
