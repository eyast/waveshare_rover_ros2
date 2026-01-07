"""
Calibration Tools for WAVE ROVER
================================

This module provides calibration tools for the WAVE ROVER robot.

Motor Calibration
-----------------
The WAVE ROVER doesn't have wheel encoders, so we use gain-based control.
The robot executes test movements and you measure the actual motion to 
compute correction gains.

>>> from pyrover.calibration import MotorCalibrator
>>> cal = MotorCalibrator(rover)
>>> result = cal.run_linear_test(command_speed=0.3, duration=2.0)
>>> # Measure actual distance traveled
>>> result.compute_gain(measured_distance=0.45)  # meters

Command Line Tools
------------------
After installing the package, use these commands:

    waveshare-calibrate-motors --port /dev/serial0
"""

from .motor import (
    MotorCalibrator,
    LinearCalibrationResult,
    AngularCalibrationResult,
    FullCalibrationResult,
)


__all__ = [
    "MotorCalibrator",
    "LinearCalibrationResult", 
    "AngularCalibrationResult",
    "FullCalibrationResult",
]
