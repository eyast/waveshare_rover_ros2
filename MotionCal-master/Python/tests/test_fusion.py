"""
Unit tests for sensor fusion

Tests mahony.py AHRS algorithm
"""

import unittest
import numpy as np
from motioncal.fusion.mahony import MahonyAHRS, fusion_init, fusion_update, fusion_read
from motioncal.data.sensor_data import AccelSensor, MagSensor, GyroSensor
from motioncal.utils.constants import OVERSAMPLE_RATIO


class TestMahonyAHRS(unittest.TestCase):
    """Test Mahony AHRS sensor fusion"""

    def setUp(self):
        """Set up test fixtures"""
        self.ahrs = MahonyAHRS()

    def test_initialization(self):
        """Test AHRS initializes to identity quaternion"""
        self.assertEqual(self.ahrs.q0, 1.0)
        self.assertEqual(self.ahrs.q1, 0.0)
        self.assertEqual(self.ahrs.q2, 0.0)
        self.assertEqual(self.ahrs.q3, 0.0)

    def test_init_resets_quaternion(self):
        """Test that init() resets the quaternion"""
        # Modify quaternion
        self.ahrs.q0 = 0.5
        self.ahrs.q1 = 0.5
        
        # Reset
        self.ahrs.init()
        
        # Should be identity (after first call, first flag is False)
        # Integral feedback should be zeroed
        self.assertEqual(self.ahrs.integralFBx, 0.0)
        self.assertEqual(self.ahrs.integralFBy, 0.0)
        self.assertEqual(self.ahrs.integralFBz, 0.0)

    def test_update_with_stationary_sensor(self):
        """Test update with stationary sensor (no rotation)"""
        # Stationary sensor: accel = [0, 0, 1g], gyro = 0, mag = [1, 0, 0]
        gx = gy = gz = 0.0  # No rotation
        ax, ay, az = 0.0, 0.0, 1.0  # Gravity down
        mx, my, mz = 1.0, 0.0, 0.0  # Magnetic north
        
        # Update multiple times
        for _ in range(100):
            self.ahrs.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        
        # Quaternion should stay close to identity
        q = self.ahrs.read()
        self.assertGreater(abs(q.q0), 0.9)  # Mostly identity

    def test_update_imu_mode(self):
        """Test 6-DOF IMU mode (no magnetometer)"""
        # No magnetometer (all zeros)
        gx = gy = gz = 0.0
        ax, ay, az = 0.0, 0.0, 1.0
        mx, my, mz = 0.0, 0.0, 0.0
        
        # Should not crash
        self.ahrs.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        
        # Should still have valid quaternion
        q = self.ahrs.read()
        magnitude = np.sqrt(q.q0**2 + q.q1**2 + q.q2**2 + q.q3**2)
        self.assertAlmostEqual(magnitude, 1.0, places=5)

    def test_quaternion_normalization(self):
        """Test that quaternion stays normalized"""
        # Random gyro input
        gx = 0.1
        gy = 0.05
        gz = -0.03
        ax, ay, az = 0.0, 0.1, 0.99
        mx, my, mz = 0.9, 0.1, 0.0
        
        # Update many times
        for _ in range(1000):
            self.ahrs.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        
        # Quaternion should remain normalized
        q = self.ahrs.read()
        magnitude = np.sqrt(q.q0**2 + q.q1**2 + q.q2**2 + q.q3**2)
        self.assertAlmostEqual(magnitude, 1.0, places=4)

    def test_inv_sqrt(self):
        """Test fast inverse square root"""
        # Test some known values
        result = MahonyAHRS._inv_sqrt(4.0)
        self.assertAlmostEqual(result, 0.5, places=3)
        
        result = MahonyAHRS._inv_sqrt(1.0)
        self.assertAlmostEqual(result, 1.0, places=3)
        
        result = MahonyAHRS._inv_sqrt(9.0)
        self.assertAlmostEqual(result, 1.0/3.0, places=3)


class TestFusionInterface(unittest.TestCase):
    """Test fusion module interface"""

    def setUp(self):
        """Set up test fixtures"""
        fusion_init()

    def test_fusion_init(self):
        """Test fusion initialization"""
        # Should not crash
        fusion_init()
        
        # Should return identity quaternion initially
        q = fusion_read()
        self.assertAlmostEqual(q.q0, 1.0, places=5)

    def test_fusion_update(self):
        """Test fusion update with sensor data"""
        # Create sensor data
        accel = AccelSensor(
            Gp=np.array([0.0, 0.0, 1.0], dtype=np.float32),
            GpFast=np.array([0.0, 0.0, 1.0], dtype=np.float32)
        )
        
        mag = MagSensor(
            Bc=np.array([50.0, 0.0, 0.0], dtype=np.float32),
            BcFast=np.array([50.0, 0.0, 0.0], dtype=np.float32)
        )
        
        gyro = GyroSensor(
            Yp=np.array([0.0, 0.0, 0.0], dtype=np.float32),
            YpFast=np.zeros((OVERSAMPLE_RATIO, 3), dtype=np.float32)
        )
        
        # Should not crash
        fusion_update(accel, mag, gyro, None)
        
        # Should return valid quaternion
        q = fusion_read()
        magnitude = np.sqrt(q.q0**2 + q.q1**2 + q.q2**2 + q.q3**2)
        self.assertAlmostEqual(magnitude, 1.0, places=4)

    def test_fusion_with_rotation(self):
        """Test fusion with rotational motion"""
        fusion_init()
        
        # Simulate rotation around Z axis
        for i in range(100):
            angle = (i / 100.0) * 0.1  # Small rotation
            
            accel = AccelSensor(
                Gp=np.array([0.0, 0.0, 1.0], dtype=np.float32),
                GpFast=np.array([0.0, 0.0, 1.0], dtype=np.float32)
            )
            
            mag = MagSensor(
                Bc=np.array([
                    np.cos(angle) * 50.0,
                    np.sin(angle) * 50.0,
                    0.0
                ], dtype=np.float32),
                BcFast=np.array([50.0, 0.0, 0.0], dtype=np.float32)
            )
            
            # Small gyro rotation
            gyro_val = np.array([0.0, 0.0, 5.0], dtype=np.float32)  # deg/s
            gyro = GyroSensor(
                Yp=gyro_val,
                YpFast=np.tile(gyro_val, (OVERSAMPLE_RATIO, 1))
            )
            
            fusion_update(accel, mag, gyro, None)
        
        # Should have rotated
        q = fusion_read()
        # Quaternion should still be normalized
        magnitude = np.sqrt(q.q0**2 + q.q1**2 + q.q2**2 + q.q3**2)
        self.assertAlmostEqual(magnitude, 1.0, places=3)


if __name__ == '__main__':
    unittest.main()
