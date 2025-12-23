"""
Sensor data structures

Ports the data structures from imuread.h:
- Point_t → Point3D
- ImuData
- Quaternion_t → Quaternion
- YawPitchRoll
- OffsetsCalibrationData
- SoftIronCalibrationData
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class Point3D:
    """3D point structure (ports Point_t from imuread.h)"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __post_init__(self):
        """Ensure float32 precision to match C float"""
        self.x = np.float32(self.x)
        self.y = np.float32(self.y)
        self.z = np.float32(self.z)


@dataclass
class YawPitchRoll:
    """Orientation angles in degrees (ports YawPitchRoll from imuread.h)"""
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    def __post_init__(self):
        """Ensure float32 precision to match C float"""
        self.yaw = np.float32(self.yaw)
        self.pitch = np.float32(self.pitch)
        self.roll = np.float32(self.roll)


@dataclass
class ImuData:
    """
    Combined IMU sensor readings (ports ImuData from imuread.h)

    Contains accelerometer, gyroscope, and magnetometer readings.
    """
    accelerometer: Point3D
    gyroscope: Point3D
    magnetometer: Point3D


@dataclass
class Quaternion:
    """
    Quaternion orientation (ports Quaternion_t from imuread.h)

    Convention: q0 is the scalar (w) component
    """
    q0: float = 1.0  # w (scalar part)
    q1: float = 0.0  # x
    q2: float = 0.0  # y
    q3: float = 0.0  # z

    def __post_init__(self):
        """Ensure float32 precision to match C float"""
        self.q0 = np.float32(self.q0)
        self.q1 = np.float32(self.q1)
        self.q2 = np.float32(self.q2)
        self.q3 = np.float32(self.q3)


@dataclass
class OffsetsCalibrationData:
    """
    Calibration offsets (ports OffsetsCalibrationData from imuread.h)

    Used for Cal1: message verification
    """
    offsetData: np.ndarray  # shape (9,) - accel(3) + mag(3) + soft_iron(3)
    calMag: float = 0.0  # geomagnetic field magnitude

    def __post_init__(self):
        """Ensure float32 precision"""
        if isinstance(self.offsetData, (list, tuple)):
            self.offsetData = np.array(self.offsetData, dtype=np.float32)
        elif self.offsetData.dtype != np.float32:
            self.offsetData = self.offsetData.astype(np.float32)
        self.calMag = np.float32(self.calMag)


@dataclass
class SoftIronCalibrationData:
    """
    Soft iron calibration matrix (ports SoftIronCalibrationData from imuread.h)

    Used for Cal2: message verification
    """
    softIronData: np.ndarray  # shape (9,) - flattened 3x3 matrix

    def __post_init__(self):
        """Ensure float32 precision"""
        if isinstance(self.softIronData, (list, tuple)):
            self.softIronData = np.array(self.softIronData, dtype=np.float32)
        elif self.softIronData.dtype != np.float32:
            self.softIronData = self.softIronData.astype(np.float32)


# Sensor structure definitions (from imuread.h)
@dataclass
class AccelSensor:
    """Accelerometer sensor structure (ports AccelSensor_t from imuread.h)"""
    Gp: np.ndarray  # slow (25Hz) averaged readings (g), shape (3,)
    GpFast: np.ndarray  # fast (200Hz) readings (g), shape (3,)

    def __post_init__(self):
        """Ensure float32 precision and correct shapes"""
        self.Gp = np.asarray(self.Gp, dtype=np.float32)
        self.GpFast = np.asarray(self.GpFast, dtype=np.float32)


@dataclass
class MagSensor:
    """Magnetometer sensor structure (ports MagSensor_t from imuread.h)"""
    Bc: np.ndarray  # slow (25Hz) averaged calibrated readings (µT), shape (3,)
    BcFast: np.ndarray  # fast (200Hz) calibrated readings (µT), shape (3,)

    def __post_init__(self):
        """Ensure float32 precision and correct shapes"""
        self.Bc = np.asarray(self.Bc, dtype=np.float32)
        self.BcFast = np.asarray(self.BcFast, dtype=np.float32)


@dataclass
class GyroSensor:
    """Gyroscope sensor structure (ports GyroSensor_t from imuread.h)"""
    Yp: np.ndarray  # raw gyro sensor output (deg/s), shape (3,)
    YpFast: np.ndarray  # fast (200Hz) readings, shape (OVERSAMPLE_RATIO, 3)

    def __post_init__(self):
        """Ensure float32 precision and correct shapes"""
        self.Yp = np.asarray(self.Yp, dtype=np.float32)
        self.YpFast = np.asarray(self.YpFast, dtype=np.float32)
