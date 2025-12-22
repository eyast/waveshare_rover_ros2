"""
Command Types and Enums for Waveshare WAVE ROVER
================================================

This module defines the JSON command protocol used to communicate
with the WAVE ROVER via serial.

Protocol Overview
-----------------
The WAVE ROVER uses JSON commands over serial (115200 baud by default).
Each command is a JSON object with at minimum a "T" field indicating
the command type.

Example commands:
    {"T":1,"L":0.3,"R":0.3}     # Move forward at 30% speed
    {"T":126}                    # Request IMU data
    {"T":130}                    # Request chassis info

Reference: https://www.waveshare.com/wiki/WAVE_ROVER
"""

from enum import IntEnum


class CommandType(IntEnum):
    """
    JSON command type identifiers (T values).
    
    These are the "T" values used in the JSON protocol to identify
    which command is being sent or what type of response is received.
    """
    # =========================================================================
    # Motion Control (T: 1-20)
    # =========================================================================
    SPEED_CTRL = 1
    """
    Set wheel speeds directly.
    
    Format: {"T":1, "L":<left_speed>, "R":<right_speed>}
    Speed range: -0.5 to 0.5 (0.5 = 100% PWM)
    
    This is the recommended command for WAVE ROVER (no encoders).
    """
    
    SET_MOTOR_PID = 2
    """
    Configure motor PID parameters (UGV01 only - has encoders).
    
    Format: {"T":2, "P":<kp>, "I":<ki>, "D":<kd>, "L":<limit>}
    """
    
    PWM_INPUT = 11
    """
    Direct PWM control.
    
    Format: {"T":11, "L":<left_pwm>, "R":<right_pwm>}
    PWM range: -255 to 255
    
    Note: DC gear motors have poor low-speed characteristics.
    Use SPEED_CTRL (T=1) instead for normal operation.
    """
    
    ROS_CTRL = 13
    """
    ROS-style velocity control (UGV01 only - has encoders).
    
    Format: {"T":13, "X":<linear_x>, "Z":<angular_z>}
    Units: m/s and rad/s
    """
    
    # =========================================================================
    # OLED Display (T: 3, -3)
    # =========================================================================
    OLED_CTRL = 3
    """
    Display text on OLED screen.
    
    Format: {"T":3, "lineNum":<0-3>, "Text":"<text>"}
    """
    
    OLED_DEFAULT = -3
    """
    Reset OLED to default display (shows robot info).
    
    Format: {"T":-3}
    """
    
    # =========================================================================
    # Module Type (T: 4)
    # =========================================================================
    MODULE_TYPE = 4
    """
    Set external module type.
    
    Format: {"T":4, "cmd":<module_type>}
    Values: 0=None, 1=RoArm-M2, 3=Gimbal
    """
    
    # =========================================================================
    # IMU & Sensors (T: 126-129)
    # =========================================================================
    GET_IMU_DATA = 126
    """
    Request full IMU data.
    
    Format: {"T":126}
    
    Response includes:
    - r, p, y: roll, pitch, yaw (degrees)
    - ax, ay, az: accelerometer (milli-g)
    - gx, gy, gz: gyroscope (deg/s)
    - mx, my, mz: magnetometer
    - temp: temperature (C)
    """
    
    CALI_IMU_STEP = 127
    """
    Start IMU calibration (reserved interface).
    
    Format: {"T":127}
    """
    
    GET_IMU_OFFSET = 128
    """
    Get current IMU offset values.
    
    Format: {"T":128}
    """
    
    SET_IMU_OFFSET = 129
    """
    Set IMU offset values (reserved interface).
    
    Format: {"T":129, "ax":<>, "ay":<>, "az":<>, "gx":<>, "gy":<>, "gz":<>}
    """
    
    # =========================================================================
    # Chassis Feedback (T: 130-143)
    # =========================================================================
    BASE_FEEDBACK = 130
    """
    Request chassis feedback (voltage, current, speeds).
    
    Format: {"T":130}
    
    Response includes:
    - v: voltage
    - i: current
    - L, R: wheel speeds
    """
    
    BASE_FEEDBACK_FLOW = 131
    """
    Enable/disable continuous serial feedback.
    
    Format: {"T":131, "cmd":<0|1>}
    
    When enabled, the chassis continuously streams feedback data
    without requiring polling. Useful for ROS integration.
    """
    
    IO_PWM_CTRL = 132
    """
    Set PWM output for IO4 and IO5 pins.
    
    Format: {"T":132, "IO4":<0-255>, "IO5":<0-255>}
    """
    
    GIMBAL_CTRL = 133
    """
    Control pan-tilt gimbal.
    
    Format: {"T":133, "X":<angle>, "Y":<angle>, "SPD":<speed>, "ACC":<accel>}
    """
    
    SERIAL_ECHO = 143
    """
    Enable/disable serial command echo.
    
    Format: {"T":143, "cmd":<0|1>}
    
    When enabled, all sent commands are echoed back.
    """
    
    # =========================================================================
    # ESP-NOW Communication (T: 301-306)
    # =========================================================================
    ESPNOW_MODE = 301
    """
    Enable/disable ESP-NOW command receiving.
    
    Format: {"T":301, "mode":<0|3>}
    """
    
    ESPNOW_ADD_PEER = 303
    """
    Add a peer for ESP-NOW communication.
    
    Format: {"T":303, "mac":"<MAC_ADDRESS>"}
    Example: {"T":303, "mac":"CC:DB:A7:5C:1C:40"}
    """
    
    ESPNOW_DEL_PEER = 304
    """
    Remove a peer from ESP-NOW communication.
    
    Format: {"T":304, "mac":"<MAC_ADDRESS>"}
    """
    
    ESPNOW_MULTICAST = 305
    """
    Send command to all added peers (multicast).
    
    Format: {"T":305, "dev":0, "b":0, "s":0, "e":0, "h":0, "cmd":1, "megs":"<json>"}
    """
    
    ESPNOW_UNICAST = 306
    """
    Send command to specific peer (unicast).
    
    Format: {"T":306, "mac":"<MAC>", "dev":0, "b":0, "s":0, "e":0, "h":0, "cmd":1, "megs":"<json>"}
    """
    
    # =========================================================================
    # WiFi Configuration (T: 401-408)
    # =========================================================================
    WIFI_MODE = 401
    """
    Set WiFi operating mode.
    
    Format: {"T":401, "cmd":<mode>}
    Modes: 0=OFF, 1=AP, 2=STA, 3=AP+STA
    """
    
    WIFI_AP_CONFIG = 403
    """
    Configure WiFi Access Point.
    
    Format: {"T":403, "ssid":"<name>", "password":"<pass>"}
    """
    
    WIFI_STA_CONFIG = 404
    """
    Configure WiFi Station mode (connect to network).
    
    Format: {"T":404, "sta_ssid":"<name>", "sta_password":"<pass>",
             "ap_ssid":"<fallback>", "ap_password":"<fallback_pass>"}
    """
    
    WIFI_INFO = 405
    """
    Get current WiFi configuration.
    
    Format: {"T":405}
    """
    
    WIFI_CONFIG_CREATE = 406
    """
    Save WiFi settings to config file.
    
    Format: {"T":406}
    """
    
    WIFI_CONFIG_CREATE_NEW = 407
    """
    Create new WiFi config file.
    
    Format: {"T":407}
    """
    
    WIFI_DISCONNECT = 408
    """
    Disconnect WiFi.
    
    Format: {"T":408}
    """
    
    # =========================================================================
    # System (T: 600)
    # =========================================================================
    REBOOT = 600
    """
    Reboot the ESP32.
    
    Format: {"T":600}
    """


class ModuleType(IntEnum):
    """External module types for T=4 command."""
    NONE = 0
    ROARM_M2 = 1
    GIMBAL = 3


class WiFiMode(IntEnum):
    """WiFi operating modes for T=401 command."""
    OFF = 0
    AP = 1      # Access Point mode (robot creates network)
    STA = 2     # Station mode (robot connects to network)
    AP_STA = 3  # Both AP and Station
