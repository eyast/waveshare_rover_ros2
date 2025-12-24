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
    
    # =========================================================================
    # System (T: 600)
    # =========================================================================
    REBOOT = 600
    """
    Reboot the ESP32.
    
    Format: {"T":600}
    """

    # =========================================================================
    # MEssage Format
    # =========================================================================
    IMU_TYPE = 600
    """
    Reboot the ESP32.
    
    Format: {"T":600}
    """

class ModuleType(IntEnum):
    """External module types for T=4 command."""
    NONE = 0
    ROARM_M2 = 1
    GIMBAL = 3

