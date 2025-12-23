"""
Apply calibration to magnetometer readings

Ports apply_calibration from visualize.c
"""

import numpy as np
from ..utils.constants import UT_PER_COUNT
from ..data.sensor_data import Point3D


def apply_calibration(rawx, rawy, rawz, magcal):
    """
    Apply magnetic calibration to raw magnetometer readings

    Ports: apply_calibration from visualize.c

    This function:
    1. Converts raw counts to µT
    2. Subtracts hard iron offset (V)
    3. Applies soft iron correction matrix (invW)

    Args:
        rawx: Raw X-axis magnetometer reading (int16 counts)
        rawy: Raw Y-axis magnetometer reading (int16 counts)
        rawz: Raw Z-axis magnetometer reading (int16 counts)
        magcal: MagCalibration instance with V and invW

    Returns:
        Point3D: Calibrated magnetometer reading in µT
    """
    # Convert raw counts to µT and subtract hard iron offset
    x = (np.float32(rawx) * UT_PER_COUNT) - magcal.V[0]
    y = (np.float32(rawy) * UT_PER_COUNT) - magcal.V[1]
    z = (np.float32(rawz) * UT_PER_COUNT) - magcal.V[2]

    # Apply soft iron correction matrix (invW)
    # This corrects for distortions in the magnetic field due to
    # ferromagnetic materials near the sensor
    out_x = x * magcal.invW[0, 0] + y * magcal.invW[0, 1] + z * magcal.invW[0, 2]
    out_y = x * magcal.invW[1, 0] + y * magcal.invW[1, 1] + z * magcal.invW[1, 2]
    out_z = x * magcal.invW[2, 0] + y * magcal.invW[2, 1] + z * magcal.invW[2, 2]

    return Point3D(out_x, out_y, out_z)
