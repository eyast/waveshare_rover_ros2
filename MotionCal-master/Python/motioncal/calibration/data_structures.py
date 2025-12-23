"""
Magnetic calibration data structures

Ports MagCalibration_t from imuread.h
"""

import numpy as np
from ..utils.constants import MAGBUFFSIZE


class MagCalibration:
    """
    Magnetic calibration state and buffer (ports MagCalibration_t from imuread.h)

    This structure holds all state for the magnetic calibration algorithms,
    including the calibration buffer, current calibration, and trial calibration.
    """

    def __init__(self):
        """Initialize magnetic calibration structure"""

        # Current calibration (accepted values)
        self.V = np.zeros(3, dtype=np.float32)  # hard iron offset (µT)
        self.invW = np.zeros((3, 3), dtype=np.float32)  # inverse soft iron matrix
        self.B = np.float32(0.0)  # geomagnetic field magnitude (µT)
        self.FourBsq = np.float32(0.0)  # 4*B*B (µT^2)
        self.FitError = np.float32(0.0)  # fit error (%)
        self.FitErrorAge = np.float32(0.0)  # aged fit error (%)

        # Trial calibration (candidate values being tested)
        self.trV = np.zeros(3, dtype=np.float32)  # trial hard iron offset (µT)
        self.trinvW = np.zeros((3, 3), dtype=np.float32)  # trial inverse soft iron matrix
        self.trB = np.float32(0.0)  # trial geomagnetic field magnitude (µT)
        self.trFitErrorpc = np.float32(0.0)  # trial fit error (%)

        # Ellipsoid matrices
        self.A = np.zeros((3, 3), dtype=np.float32)  # ellipsoid matrix A
        self.invA = np.zeros((3, 3), dtype=np.float32)  # inverse of ellipsoid matrix A

        # Scratch matrices for calibration algorithms
        self.matA = np.zeros((10, 10), dtype=np.float32)  # scratch 10x10 matrix
        self.matB = np.zeros((10, 10), dtype=np.float32)  # scratch 10x10 matrix
        self.vecA = np.zeros(10, dtype=np.float32)  # scratch 10x1 vector
        self.vecB = np.zeros(4, dtype=np.float32)  # scratch 4x1 vector

        # Calibration state
        self.ValidMagCal = np.int8(0)  # 0, 4, 7, or 10 denoting solver used

        # Magnetometer buffer (650 uncalibrated readings)
        # BpFast[axis][sample] where axis: 0=x, 1=y, 2=z
        self.BpFast = np.zeros((3, MAGBUFFSIZE), dtype=np.int16)
        self.valid = np.zeros(MAGBUFFSIZE, dtype=np.int8)  # 1=has data, 0=empty
        self.MagBufferCount = np.int16(0)  # number of valid readings

        # Initialize to default values
        self.reset()

    def reset(self):
        """Reset calibration to initial state (called by raw_data_reset)"""
        # Initialize hard iron offset with initial guess
        self.V[0] = np.float32(0.0)
        self.V[1] = np.float32(0.0)
        self.V[2] = np.float32(80.0)  # initial Z-axis guess

        # Initialize soft iron to identity matrix
        self.invW.fill(0.0)
        self.invW[0, 0] = np.float32(1.0)
        self.invW[1, 1] = np.float32(1.0)
        self.invW[2, 2] = np.float32(1.0)

        # Initialize field and error estimates
        self.B = np.float32(50.0)
        self.FitError = np.float32(100.0)
        self.FitErrorAge = np.float32(100.0)

        # Clear buffers
        self.BpFast.fill(0)
        self.valid.fill(0)
        self.MagBufferCount = np.int16(0)
        self.ValidMagCal = np.int8(0)

        # Clear trial values
        self.trV.fill(0.0)
        self.trinvW.fill(0.0)
        self.trB = np.float32(0.0)
        self.trFitErrorpc = np.float32(0.0)

        # Clear matrices
        self.A.fill(0.0)
        self.invA.fill(0.0)
        self.matA.fill(0.0)
        self.matB.fill(0.0)
        self.vecA.fill(0.0)
        self.vecB.fill(0.0)
        self.FourBsq = np.float32(0.0)


# Global magnetic calibration instance (matches C extern MagCalibration_t magcal)
magcal = MagCalibration()
