"""
Waveshare PyRover Python Library
====================================

A pure Python library for controlling Waveshare PyRover robots.
No ROS2 dependencies - can be used standalone or with ROS2 via the
rover_ros package.

Main Classes
------------
PyRover
    Main controller class for serial communication with the robot.
    
Data Types
----------
IMUData
    Full 9-axis IMU sensor data from the robot.
RealTimeData
    Continuous feedback data (when streaming is enabled).
ChassisInfo
    Battery voltage, current, and wheel speeds.
BatteryEstimator
    Estimate battery state of charge from voltage.

Command Enums
-------------
CommandType
    JSON command type identifiers.
WiFiMode
    WiFi operating modes.
ModuleType
    External module types.


Calibration Subpackage
----------------------
The `calibration` subpackage provides calibration tools:

>>> from pyrover.calibration import MotorCalibrator

Command-line tools are also available after installation:

    pyrover-calibrate-motors --port /dev/serial0

Quick Start
-----------
>>> from pyrover import PyRover
>>> 
>>> # Using context manager (recommended)
>>> with PyRover('/dev/serial0') as rover:
...     rover.move(0.3, 0.3)  # Forward
...     time.sleep(2)
...     rover.stop()
>>> 
>>> # Or manual connection
>>> rover = PyRover('/dev/serial0')
>>> rover.connect()
>>> imu = rover.get_imu_data()
>>> print(f"Yaw: {imu.yaw:.1f}°")
>>> rover.disconnect()
"""

from .rover import PyRover
from .data_types import (
    IMUData,
    ChassisInfo,
    BatteryEstimator,
)
from .commands import (
    CommandType,
    WiFiMode,
    ModuleType,
)
from .tools import quaternion_from_euler, wrap_angle

__version__ = "0.1.0"
__author__ = "Eyas Taifour"

__all__ = [
    # Main class
    "PyRover",
    
    # Data types
    "IMUData",
    "ChassisInfo",
    "BatteryEstimator",
    
    # Command enums
    "CommandType",
    "WiFiMode",
    "ModuleType",

    # Tools
    "quaternion_from_euler",
    "wrap_angle"
]
