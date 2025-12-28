"""
Command Protocol for Waveshare WAVE ROVER (New Line-Based Driver)
=================================================================

This module defines the line-based protocol used to communicate
with the WAVE ROVER via serial.

Protocol Overview
-----------------
The new driver uses simple line-based commands instead of JSON.
Each line has a prefix followed by colon and comma-separated values.

Output (Device → Host):
    I:yaw,pitch,roll,temp,ax,ay,az,gx,gy,gz,mx,my,mz  - IMU telemetry
    P:voltage,current,power,shunt                      - Power data
    Raw:ax,ay,az,gx,gy,gz,mx,my,mz                    - MotionCal raw
    Ori:yaw,pitch,roll                                 - MotionCal orientation
    S:module,message                                   - System messages
    A:command[,details]                                - Acknowledgments
    E:source,message                                   - Errors

Input (Host → Device):
    M:left,right      - Motor speeds (-255 to 255)
    STOP              - Stop motors
    ESTOP             - Emergency stop (requires ENABLE to resume)
    ENABLE            - Re-enable after ESTOP
    HB                - Heartbeat (reset timeout)
    STREAM:ON/OFF     - Enable/disable telemetry
    FMT:RAW/IMU       - Switch output format (MotionCal vs telemetry)
    CAL:G/A/ALL       - Calibrate gyro/accel/all
    BETA:value        - Set Madgwick filter beta
    FAST:duration     - Start fast convergence mode
    INIT              - Re-initialize filter from sensors
    WS:ssid,pass,ip[,port] - Connect WiFi and WebSocket
    WS:OFF            - Disconnect WebSocket
    STATUS            - Request system status
    REBOOT            - Restart device
"""


class OutputPrefix:
    """Prefixes for messages from device to host."""
    IMU = "I:"           # IMU telemetry
    POWER = "P:"         # Power data
    RAW = "Raw:"         # MotionCal raw sensor data
    ORI = "Ori:"         # MotionCal orientation
    SYSTEM = "S:"        # System messages
    ACK = "A:"           # Command acknowledgments
    ERROR = "E:"         # Error messages
    DEBUG = "D:"         # Debug output


class StreamFormat:
    """Stream format options for FMT command."""
    RAW = "RAW"          # MotionCal format (Raw: and Ori:)
    IMU = "IMU"          # Telemetry format (I:)


# Motor limits
MAX_PWM = 255
MIN_PWM = -255

# Default timeout (robot stops if no heartbeat/command)
HEARTBEAT_TIMEOUT_S = 3.0