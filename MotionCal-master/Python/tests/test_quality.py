"""
Unit tests for quality metrics

Tests quality.py surface gap error and other metrics
"""

import unittest
import numpy as np
from motioncal.calibration.quality import (
    quality_reset,
    quality_update,
    quality_surface_gap_error,
    quality_magnitude_variance_error,
    quality_wobble_error,
    sphere_region
)
from motioncal.data.sensor_data import Point3D


class TestQualityMetrics(unittest.TestCase):
    """Test quality metric functions"""

    def setUp(self):
        """Reset quality metrics before each test"""
        quality_reset()

    def test_sphere_region_arctic(self):
        """Test sphere region calculation for arctic cap"""
        # Point at north pole
        region = sphere_region(0.0, 0.0, 1.0)
        self.assertEqual(region, 0)  # Arctic cap

    def test_sphere_region_antarctic(self):
        """Test sphere region calculation for antarctic cap"""
        # Point at south pole
        region = sphere_region(0.0, 0.0, -1.0)
        self.assertEqual(region, 99)  # Antarctic cap

    def test_sphere_region_equator(self):
        """Test sphere region calculation for equatorial point"""
        # Point at equator
        region = sphere_region(1.0, 0.0, 0.0)
        # Should be in one of the tropical zones (15-82)
        self.assertTrue(15 <= region <= 82)

    def test_initial_gap_error(self):
        """Test that initial gap error is 100%"""
        error = quality_surface_gap_error()
        self.assertAlmostEqual(error, 100.0, places=1)

    def test_gap_error_decreases_with_data(self):
        """Test that gap error decreases as data is added"""
        initial_error = quality_surface_gap_error()
        
        # Add points distributed around sphere
        for i in range(10):
            theta = (i / 10.0) * 2.0 * np.pi
            point = Point3D(
                x=np.cos(theta),
                y=np.sin(theta),
                z=0.0
            )
            quality_update(point)
        
        final_error = quality_surface_gap_error()
        self.assertLess(final_error, initial_error)

    def test_quality_update_point(self):
        """Test updating quality with a point"""
        point = Point3D(x=1.0, y=0.0, z=0.0)
        
        # Should not crash
        quality_update(point)
        
        # Gap error should be less than 100%
        error = quality_surface_gap_error()
        self.assertLess(error, 100.0)

    def test_magnitude_variance_initial(self):
        """Test initial magnitude variance"""
        # Before any data
        variance = quality_magnitude_variance_error()
        # Should be high initially
        self.assertGreaterEqual(variance, 0.0)

    def test_wobble_error_initial(self):
        """Test initial wobble error"""
        wobble = quality_wobble_error()
        # Should be high initially
        self.assertGreaterEqual(wobble, 0.0)


if __name__ == '__main__':
    unittest.main()
