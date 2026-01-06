"""
PyRover - Waveshare WAVE ROVER Python Library
=============================================

A Python library for controlling Waveshare WAVE ROVER robots
using the new line-based serial protocol.

Example:
    >>> from pyrover import PyRover
    >>> 
    >>> with PyRover('/dev/serial0') as rover:
    ...     rover.move(100, 100)  # Forward
    ...     rover.stop()
"""

from .rover import PyRover, RoverCallbacks
from .data_types import (
    IMUData,
    PowerData,
    RawSensorData,
    Orientation,
    SystemMessage,
    BatteryEstimator, # Backwards compatibility
)
from .commands import (
    OutputPrefix,
    StreamFormat,
    MAX_PWM,
    MIN_PWM,
)

from pyrover.tools.mathtools import (quaternion_from_euler, wrap_angle)

from pyrover.tools.utilities import log_exceptions

__version__ = "2.0.0"
__all__ = [
    "PyRover",
    "RoverCallbacks",
    "IMUData",
    "PowerData",
    "RawSensorData",
    "Orientation",
    "SystemMessage",
    "BatteryEstimator",
    "OutputPrefix",
    "StreamFormat",
    "MAX_PWM",
    "MIN_PWM",
    "quaternion_from_euler",
    "wrap_angle",
    "log_exceptions"
]