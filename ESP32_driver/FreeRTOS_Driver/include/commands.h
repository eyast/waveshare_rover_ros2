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
 *   CALIB:ON        - Apply hardcoded calibration adjustment
 *                     to the data streamed in RAW format
 *                     to Magnetomer (Soft/Hard Iron)
 *                        Gyroscope (Bias)
 *                        Accelerometer (Bias and Calibration 
 *                                       Matrix)
 *   CALIB:OFF       - Disable all the adjustments above
 *   CALIB:STATUS    - Get Calibration status
 *                     0 - Calibration is not applied
 *                     1 - Calibration is applied
 *   
 *   WDT:STATUS      - Show only task health status
 *   WDT:STACK       - Show only stack usage
 * 
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
