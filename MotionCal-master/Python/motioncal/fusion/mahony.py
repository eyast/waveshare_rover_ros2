"""
Mahony AHRS algorithm

Ports mahony.c - Madgwick's implementation of Mayhony's AHRS algorithm

Reference:
http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

Algorithm paper:
http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=4608934

Original Authors:
- 29/09/2011 SOH Madgwick - Initial release
- 02/10/2011 SOH Madgwick - Optimised for reduced CPU load

License: GNU General Public License
"""

import numpy as np
import struct
from ..utils.constants import OVERSAMPLE_RATIO, SENSORFS
from ..data.sensor_data import Quaternion


# Algorithm constants
TWO_KP_DEF = np.float32(2.0 * 0.02)  # 2 * proportional gain
TWO_KI_DEF = np.float32(2.0 * 0.0)   # 2 * integral gain (disabled)
INV_SAMPLE_RATE = np.float32(1.0 / SENSORFS)


class MahonyAHRS:
    """
    Mahony AHRS sensor fusion algorithm

    Fuses accelerometer, gyroscope, and magnetometer data to produce
    orientation quaternion.
    """

    def __init__(self):
        """Initialize Mahony AHRS state"""
        # Gains
        self.twoKp = TWO_KP_DEF  # Proportional gain
        self.twoKi = TWO_KI_DEF  # Integral gain (0 = disabled)

        # Quaternion (sensor frame relative to earth frame)
        self.q0 = np.float32(1.0)  # w
        self.q1 = np.float32(0.0)  # x
        self.q2 = np.float32(0.0)  # y
        self.q3 = np.float32(0.0)  # z

        # Integral error terms
        self.integralFBx = np.float32(0.0)
        self.integralFBy = np.float32(0.0)
        self.integralFBz = np.float32(0.0)

        # First update flag (for initial orientation capture)
        self.first = True
        self.reset_next_update = False

    def init(self):
        """
        Reset AHRS state

        Ports: mahony_init from mahony.c
        """
        self.twoKp = TWO_KP_DEF
        self.twoKi = TWO_KI_DEF

        if self.first:
            self.q0 = np.float32(1.0)
            self.q1 = np.float32(0.0)
            self.q2 = np.float32(0.0)
            self.q3 = np.float32(0.0)
            self.first = False

        self.reset_next_update = True
        self.integralFBx = np.float32(0.0)
        self.integralFBy = np.float32(0.0)
        self.integralFBz = np.float32(0.0)

    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        """
        Update AHRS with 9-DOF sensor data

        Ports: mahony_update from mahony.c

        Args:
            gx, gy, gz: Gyroscope (rad/s)
            ax, ay, az: Accelerometer (any units, will be normalized)
            mx, my, mz: Magnetometer (any units, will be normalized)
        """
        # Use IMU algorithm if magnetometer invalid (avoids NaN)
        if (mx == 0.0) and (my == 0.0) and (mz == 0.0):
            self.update_imu(gx, gy, gz, ax, ay, az)
            return

        # Compute feedback only if accelerometer valid (avoids NaN)
        if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)):
            # Normalize accelerometer
            recipNorm = self._inv_sqrt(ax * ax + ay * ay + az * az)
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm

            # Normalize magnetometer
            recipNorm = self._inv_sqrt(mx * mx + my * my + mz * mz)
            mx *= recipNorm
            my *= recipNorm
            mz *= recipNorm

            # Auxiliary variables to avoid repeated arithmetic
            q0q0 = self.q0 * self.q0
            q0q1 = self.q0 * self.q1
            q0q2 = self.q0 * self.q2
            q0q3 = self.q0 * self.q3
            q1q1 = self.q1 * self.q1
            q1q2 = self.q1 * self.q2
            q1q3 = self.q1 * self.q3
            q2q2 = self.q2 * self.q2
            q2q3 = self.q2 * self.q3
            q3q3 = self.q3 * self.q3

            # Reference direction of Earth's magnetic field
            hx = np.float32(2.0) * (mx * (np.float32(0.5) - q2q2 - q3q3) +
                                    my * (q1q2 - q0q3) +
                                    mz * (q1q3 + q0q2))
            hy = np.float32(2.0) * (mx * (q1q2 + q0q3) +
                                    my * (np.float32(0.5) - q1q1 - q3q3) +
                                    mz * (q2q3 - q0q1))
            bx = np.sqrt(hx * hx + hy * hy)
            bz = np.float32(2.0) * (mx * (q1q3 - q0q2) +
                                    my * (q2q3 + q0q1) +
                                    mz * (np.float32(0.5) - q1q1 - q2q2))

            # Estimated direction of gravity and magnetic field
            halfvx = q1q3 - q0q2
            halfvy = q0q1 + q2q3
            halfvz = q0q0 - np.float32(0.5) + q3q3
            halfwx = bx * (np.float32(0.5) - q2q2 - q3q3) + bz * (q1q3 - q0q2)
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3)
            halfwz = bx * (q0q2 + q1q3) + bz * (np.float32(0.5) - q1q1 - q2q2)

            # Error is cross product between estimated and measured directions
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy)
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz)
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx)

            # Compute and apply integral feedback if enabled
            if self.twoKi > 0.0:
                # Integral error scaled by Ki
                self.integralFBx += self.twoKi * halfex * INV_SAMPLE_RATE
                self.integralFBy += self.twoKi * halfey * INV_SAMPLE_RATE
                self.integralFBz += self.twoKi * halfez * INV_SAMPLE_RATE
                gx += self.integralFBx  # Apply integral feedback
                gy += self.integralFBy
                gz += self.integralFBz
            else:
                # Prevent integral windup
                self.integralFBx = np.float32(0.0)
                self.integralFBy = np.float32(0.0)
                self.integralFBz = np.float32(0.0)

            # Apply proportional feedback
            if self.reset_next_update:
                # Stronger correction on first update after reset
                gx += np.float32(2.0) * halfex
                gy += np.float32(2.0) * halfey
                gz += np.float32(2.0) * halfez
                self.reset_next_update = False
            else:
                gx += self.twoKp * halfex
                gy += self.twoKp * halfey
                gz += self.twoKp * halfez

        # Integrate rate of change of quaternion
        gx *= (np.float32(0.5) * INV_SAMPLE_RATE)
        gy *= (np.float32(0.5) * INV_SAMPLE_RATE)
        gz *= (np.float32(0.5) * INV_SAMPLE_RATE)
        qa = self.q0
        qb = self.q1
        qc = self.q2
        self.q0 += (-qb * gx - qc * gy - self.q3 * gz)
        self.q1 += (qa * gx + qc * gz - self.q3 * gy)
        self.q2 += (qa * gy - qb * gz + self.q3 * gx)
        self.q3 += (qa * gz + qb * gy - qc * gx)

        # Normalize quaternion
        recipNorm = self._inv_sqrt(self.q0 * self.q0 + self.q1 * self.q1 +
                                   self.q2 * self.q2 + self.q3 * self.q3)
        self.q0 *= recipNorm
        self.q1 *= recipNorm
        self.q2 *= recipNorm
        self.q3 *= recipNorm

    def update_imu(self, gx, gy, gz, ax, ay, az):
        """
        Update AHRS with 6-DOF sensor data (no magnetometer)

        Ports: mahony_updateIMU from mahony.c

        Args:
            gx, gy, gz: Gyroscope (rad/s)
            ax, ay, az: Accelerometer (any units, will be normalized)
        """
        # Compute feedback only if accelerometer valid (avoids NaN)
        if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)):
            # Normalize accelerometer
            recipNorm = self._inv_sqrt(ax * ax + ay * ay + az * az)
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm

            # Estimated direction of gravity
            halfvx = self.q1 * self.q3 - self.q0 * self.q2
            halfvy = self.q0 * self.q1 + self.q2 * self.q3
            halfvz = self.q0 * self.q0 - np.float32(0.5) + self.q3 * self.q3

            # Error is cross product between estimated and measured gravity
            halfex = (ay * halfvz - az * halfvy)
            halfey = (az * halfvx - ax * halfvz)
            halfez = (ax * halfvy - ay * halfvx)

            # Compute and apply integral feedback if enabled
            if self.twoKi > 0.0:
                self.integralFBx += self.twoKi * halfex * INV_SAMPLE_RATE
                self.integralFBy += self.twoKi * halfey * INV_SAMPLE_RATE
                self.integralFBz += self.twoKi * halfez * INV_SAMPLE_RATE
                gx += self.integralFBx
                gy += self.integralFBy
                gz += self.integralFBz
            else:
                self.integralFBx = np.float32(0.0)
                self.integralFBy = np.float32(0.0)
                self.integralFBz = np.float32(0.0)

            # Apply proportional feedback
            gx += self.twoKp * halfex
            gy += self.twoKp * halfey
            gz += self.twoKp * halfez

        # Integrate rate of change of quaternion
        gx *= (np.float32(0.5) * INV_SAMPLE_RATE)
        gy *= (np.float32(0.5) * INV_SAMPLE_RATE)
        gz *= (np.float32(0.5) * INV_SAMPLE_RATE)
        qa = self.q0
        qb = self.q1
        qc = self.q2
        self.q0 += (-qb * gx - qc * gy - self.q3 * gz)
        self.q1 += (qa * gx + qc * gz - self.q3 * gy)
        self.q2 += (qa * gy - qb * gz + self.q3 * gx)
        self.q3 += (qa * gz + qb * gy - qc * gx)

        # Normalize quaternion
        recipNorm = self._inv_sqrt(self.q0 * self.q0 + self.q1 * self.q1 +
                                   self.q2 * self.q2 + self.q3 * self.q3)
        self.q0 *= recipNorm
        self.q1 *= recipNorm
        self.q2 *= recipNorm
        self.q3 *= recipNorm

    def read(self):
        """
        Read current orientation quaternion

        Ports: fusion_read from mahony.c

        Returns:
            Quaternion: Current orientation
        """
        return Quaternion(q0=self.q0, q1=self.q1, q2=self.q2, q3=self.q3)

    @staticmethod
    def _inv_sqrt(x):
        """
        Fast inverse square root

        Ports: invSqrt from mahony.c

        Uses the famous "Quake III" fast inverse square root algorithm
        with three Newton-Raphson iterations for improved accuracy.

        Reference: http://en.wikipedia.org/wiki/Fast_inverse_square_root

        Args:
            x: Input value

        Returns:
            float: 1 / sqrt(x)
        """
        # Convert float to int32 representation
        halfx = np.float32(0.5) * x
        i = struct.unpack('>l', struct.pack('>f', x))[0]

        # Magic bit hack
        i = 0x5f375a86 - (i >> 1)

        # Convert back to float
        y = struct.unpack('>f', struct.pack('>l', i))[0]

        # Three Newton-Raphson iterations
        y = y * (np.float32(1.5) - (halfx * y * y))
        y = y * (np.float32(1.5) - (halfx * y * y))
        y = y * (np.float32(1.5) - (halfx * y * y))

        return y


# Global instance (matches C static variables)
_mahony = MahonyAHRS()


def fusion_init():
    """
    Initialize sensor fusion

    Ports: fusion_init from mahony.c
    """
    global _mahony
    _mahony.init()


def fusion_update(accel, mag, gyro, magcal):
    """
    Update sensor fusion with new sensor data

    Ports: fusion_update from mahony.c

    Args:
        accel: AccelSensor with Gp (averaged accelerometer in G)
        mag: MagSensor with Bc (calibrated magnetometer in µT)
        gyro: GyroSensor with YpFast (gyroscope in deg/s)
        magcal: MagCalibration (not used by Mahony, for compatibility)
    """
    global _mahony

    # Convert degrees to radians
    factor = np.float32(np.pi / 180.0)

    # Get averaged accelerometer and magnetometer
    ax = accel.Gp[0]
    ay = accel.Gp[1]
    az = accel.Gp[2]
    mx = mag.Bc[0]
    my = mag.Bc[1]
    mz = mag.Bc[2]

    # Update with each gyroscope sample (OVERSAMPLE_RATIO times)
    for i in range(OVERSAMPLE_RATIO):
        gx = gyro.YpFast[i, 0] * factor
        gy = gyro.YpFast[i, 1] * factor
        gz = gyro.YpFast[i, 2] * factor
        _mahony.update(gx, gy, gz, ax, ay, az, mx, my, mz)


def fusion_read():
    """
    Read current orientation quaternion

    Ports: fusion_read from mahony.c

    Returns:
        Quaternion: Current orientation
    """
    global _mahony
    return _mahony.read()
