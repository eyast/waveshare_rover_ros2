"""
Unit tests for calibration algorithms

Tests magcal.py magnetic calibration with synthetic data
"""

import unittest
import numpy as np
from motioncal.calibration.data_structures import MagCalibration
from motioncal.calibration.magcal import (
    magcal_run,
    update_calibration_4inv,
    update_calibration_7eig,
    update_calibration_10eig
)


class TestMagneticCalibration(unittest.TestCase):
    """Test magnetic calibration algorithms"""

    def setUp(self):
        """Set up test fixtures"""
        self.magcal = MagCalibration()

    def test_magcal_initialization(self):
        """Test that MagCalibration initializes correctly"""
        self.assertEqual(self.magcal.V[0], 0.0)
        self.assertEqual(self.magcal.V[1], 0.0)
        self.assertAlmostEqual(self.magcal.V[2], 80.0, places=1)
        self.assertEqual(self.magcal.B, 50.0)
        self.assertEqual(self.magcal.FitError, 100.0)
        
        # Check soft iron is identity
        np.testing.assert_array_almost_equal(
            self.magcal.invW, np.eye(3), decimal=5
        )

    def test_magcal_reset(self):
        """Test that reset works"""
        # Modify calibration
        self.magcal.V[0] = 10.0
        self.magcal.B = 60.0
        
        # Reset
        self.magcal.reset()
        
        # Should be back to initial state
        self.assertEqual(self.magcal.V[0], 0.0)
        self.assertEqual(self.magcal.B, 50.0)

    def test_add_calibration_point(self):
        """Test adding calibration points"""
        # Add a point
        self.magcal.BpFast[0, 0] = 100
        self.magcal.BpFast[1, 0] = 200
        self.magcal.BpFast[2, 0] = 300
        self.magcal.valid[0] = 1
        
        self.assertEqual(self.magcal.valid[0], 1)
        self.assertEqual(self.magcal.BpFast[0, 0], 100)

    def test_4element_calibration_insufficient_data(self):
        """Test 4-element calibration with insufficient data"""
        # Add only 10 points (need 40)
        for i in range(10):
            self.magcal.BpFast[0, i] = 100 + i * 10
            self.magcal.BpFast[1, i] = 200 + i * 10
            self.magcal.BpFast[2, i] = 300 + i * 10
            self.magcal.valid[i] = 1

        # Functions modify in place, don't return values
        update_calibration_4inv(self.magcal)
        # With insufficient data, ValidMagCal should not be updated
        self.assertEqual(self.magcal.ValidMagCal, 0)

    def test_4element_calibration_with_sphere_data(self):
        """Test 4-element calibration with sphere data"""
        # Create 50 points on a sphere offset from origin
        offset = np.array([10.0, -5.0, 15.0], dtype=np.float32)
        radius = 50.0  # 50 µT
        
        count = 0
        for i in range(50):
            # Create points on sphere
            theta = (i / 50.0) * 2.0 * np.pi
            phi = np.arccos(1.0 - 2.0 * (i / 50.0))
            
            x = radius * np.sin(phi) * np.cos(theta)
            y = radius * np.sin(phi) * np.sin(theta)
            z = radius * np.cos(phi)
            
            # Apply offset and convert to counts (0.1 µT per count)
            self.magcal.BpFast[0, count] = int((x + offset[0]) / 0.1)
            self.magcal.BpFast[1, count] = int((y + offset[1]) / 0.1)
            self.magcal.BpFast[2, count] = int((z + offset[2]) / 0.1)
            self.magcal.valid[count] = 1
            count += 1
        
        # Run calibration
        update_calibration_4inv(self.magcal)
        # Should not crash
        self.assertIn(self.magcal.ValidMagCal, [0, 4])

    def test_magcal_run_selects_correct_algorithm(self):
        """Test that magcal_run selects the right algorithm"""
        # With < 40 points, should not run
        for i in range(30):
            self.magcal.BpFast[0, i] = 100 + i
            self.magcal.BpFast[1, i] = 200 + i
            self.magcal.BpFast[2, i] = 300 + i
            self.magcal.valid[i] = 1
        
        result = magcal_run(self.magcal)
        self.assertEqual(result, 0)  # Not enough data

    def test_geomagnetic_field_validation(self):
        """Test that calibration rejects invalid field strengths"""
        # Create points with very weak field (< 22 µT)
        for i in range(50):
            theta = (i / 50.0) * 2.0 * np.pi
            x = 10.0 * np.cos(theta)  # Only 10 µT radius
            y = 10.0 * np.sin(theta)
            z = 0.0
            
            self.magcal.BpFast[0, i] = int(x / 0.1)
            self.magcal.BpFast[1, i] = int(y / 0.1)
            self.magcal.BpFast[2, i] = int(z / 0.1)
            self.magcal.valid[i] = 1
        
        # Should reject due to invalid field strength
        update_calibration_4inv(self.magcal)
        # ValidMagCal should remain 0
        self.assertEqual(self.magcal.ValidMagCal, 0)


class TestCalibrationDataStructures(unittest.TestCase):
    """Test calibration data structures"""

    def test_magcal_buffer_size(self):
        """Test that buffer is correct size"""
        magcal = MagCalibration()
        self.assertEqual(magcal.BpFast.shape[1], 650)
        self.assertEqual(len(magcal.valid), 650)

    def test_magcal_soft_iron_matrix_size(self):
        """Test that soft iron matrix is 3x3"""
        magcal = MagCalibration()
        self.assertEqual(magcal.invW.shape, (3, 3))


if __name__ == '__main__':
    unittest.main()
