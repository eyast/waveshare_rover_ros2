"""
Unit tests for visualization transforms

Tests quaternion operations
"""

import unittest
import numpy as np
from motioncal.visualization.transforms import (
    quaternion_to_rotation_matrix,
    quaternion_to_euler,
    quaternion_normalize,
    rotate_point
)
from motioncal.data.sensor_data import Quaternion, Point3D


class TestQuaternionTransforms(unittest.TestCase):
    """Test quaternion transformation functions"""

    def test_identity_quaternion_rotation_matrix(self):
        """Test that identity quaternion gives identity matrix"""
        q = Quaternion(q0=1.0, q1=0.0, q2=0.0, q3=0.0)
        R = quaternion_to_rotation_matrix(q)
        
        expected = np.eye(3, dtype=np.float32)
        np.testing.assert_array_almost_equal(R, expected, decimal=5)

    def test_rotation_matrix_is_orthogonal(self):
        """Test that rotation matrix is orthogonal"""
        q = Quaternion(q0=0.7071, q1=0.7071, q2=0.0, q3=0.0)
        R = quaternion_to_rotation_matrix(q)
        
        # R * R^T should be identity
        I = np.dot(R, R.T)
        np.testing.assert_array_almost_equal(I, np.eye(3), decimal=4)

    def test_rotation_matrix_determinant(self):
        """Test that rotation matrix has determinant 1"""
        q = Quaternion(q0=0.5, q1=0.5, q2=0.5, q3=0.5)
        R = quaternion_to_rotation_matrix(q)
        
        det = np.linalg.det(R)
        self.assertAlmostEqual(det, 1.0, places=4)

    def test_quaternion_normalize(self):
        """Test quaternion normalization"""
        q = Quaternion(q0=1.0, q1=1.0, q2=1.0, q3=1.0)
        q_norm = quaternion_normalize(q)
        
        magnitude = np.sqrt(q_norm.q0**2 + q_norm.q1**2 + 
                           q_norm.q2**2 + q_norm.q3**2)
        self.assertAlmostEqual(magnitude, 1.0, places=6)

    def test_quaternion_to_euler_identity(self):
        """Test Euler angles for identity quaternion"""
        q = Quaternion(q0=1.0, q1=0.0, q2=0.0, q3=0.0)
        yaw, pitch, roll = quaternion_to_euler(q)
        
        # Should be close to zero
        self.assertAlmostEqual(yaw, 0.0, places=2)
        self.assertAlmostEqual(pitch, 0.0, places=2)
        self.assertAlmostEqual(roll, 0.0, places=2)

    def test_rotate_point_identity(self):
        """Test rotating point with identity matrix"""
        point = Point3D(x=1.0, y=2.0, z=3.0)
        R = np.eye(3, dtype=np.float32)
        
        x, y, z = rotate_point(point, R)
        
        self.assertAlmostEqual(x, 1.0, places=5)
        self.assertAlmostEqual(y, 2.0, places=5)
        self.assertAlmostEqual(z, 3.0, places=5)

    def test_rotate_point_90deg_z(self):
        """Test 90 degree rotation around Z axis"""
        # 90 degree rotation around Z: (1,0,0) -> (0,1,0)
        q = Quaternion(q0=0.7071, q1=0.0, q2=0.0, q3=0.7071)
        R = quaternion_to_rotation_matrix(q)
        
        point = Point3D(x=1.0, y=0.0, z=0.0)
        x, y, z = rotate_point(point, R)
        
        # Should rotate to approximately (0, 1, 0)
        self.assertAlmostEqual(x, 0.0, places=3)
        self.assertAlmostEqual(y, 1.0, places=3)
        self.assertAlmostEqual(z, 0.0, places=3)

    def test_quaternion_conjugate_reverses_rotation(self):
        """Test that conjugate quaternion reverses rotation"""
        # Original quaternion
        q = Quaternion(q0=0.7071, q1=0.7071, q2=0.0, q3=0.0)
        R = quaternion_to_rotation_matrix(q)
        
        # Conjugate (negate imaginary parts)
        q_conj = Quaternion(q0=q.q0, q1=-q.q1, q2=-q.q2, q3=-q.q3)
        R_conj = quaternion_to_rotation_matrix(q_conj)
        
        # R * R_conj should be identity
        I = np.dot(R, R_conj)
        np.testing.assert_array_almost_equal(I, np.eye(3), decimal=4)


class TestEulerAngles(unittest.TestCase):
    """Test Euler angle conversions"""

    def test_pitch_90_degrees(self):
        """Test 90 degree pitch"""
        # Pure pitch rotation  
        q = Quaternion(q0=0.7071, q1=0.7071, q2=0.0, q3=0.0)
        yaw, pitch, roll = quaternion_to_euler(q)
        
        # Should return valid angles
        self.assertGreaterEqual(yaw, -180.0)
        self.assertLessEqual(yaw, 180.0)

    def test_euler_angles_range(self):
        """Test that Euler angles are in expected range"""
        q = Quaternion(q0=0.6, q1=0.4, q2=0.5, q3=0.5)
        yaw, pitch, roll = quaternion_to_euler(q)
        
        # Yaw: -180 to 180
        self.assertGreaterEqual(yaw, -180.0)
        self.assertLessEqual(yaw, 180.0)
        
        # Pitch: -90 to 90
        self.assertGreaterEqual(pitch, -90.0)
        self.assertLessEqual(pitch, 90.0)
        
        # Roll: -180 to 180
        self.assertGreaterEqual(roll, -180.0)
        self.assertLessEqual(roll, 180.0)


if __name__ == '__main__':
    unittest.main()
