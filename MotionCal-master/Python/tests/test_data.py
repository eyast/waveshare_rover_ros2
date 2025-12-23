"""
Unit tests for data handling

Tests raw_data.py and apply_calibration.py
"""

import unittest
import numpy as np
from motioncal.data.raw_data import (
    raw_data_reset,
    choose_discard_magcal,
    add_magcal_data,
    is_float_ok,
    cal1_data,
    cal2_data
)
from motioncal.data.apply_calibration import apply_calibration
from motioncal.calibration.data_structures import MagCalibration
from motioncal.calibration.quality import quality_surface_gap_error
from motioncal.fusion.mahony import fusion_init


class TestApplyCalibration(unittest.TestCase):
    """Test calibration application"""

    def test_apply_calibration_no_offset(self):
        """Test applying calibration with no offset"""
        magcal = MagCalibration()
        magcal.V = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        magcal.invW = np.eye(3, dtype=np.float32)
        
        # Raw magnetometer reading (in counts)
        rawx, rawy, rawz = 500, 0, 0  # 50 µT in X
        
        point = apply_calibration(rawx, rawy, rawz, magcal)
        
        # Should be 50 µT (500 counts * 0.1)
        self.assertAlmostEqual(point.x, 50.0, places=1)
        self.assertAlmostEqual(point.y, 0.0, places=1)
        self.assertAlmostEqual(point.z, 0.0, places=1)

    def test_apply_calibration_with_offset(self):
        """Test applying calibration with hard iron offset"""
        magcal = MagCalibration()
        magcal.V = np.array([10.0, -5.0, 20.0], dtype=np.float32)
        magcal.invW = np.eye(3, dtype=np.float32)
        
        # Raw reading offset by hard iron
        rawx = int((50.0 + 10.0) / 0.1)  # 60 µT in counts
        rawy = int((0.0 - 5.0) / 0.1)    # -5 µT in counts
        rawz = int((0.0 + 20.0) / 0.1)   # 20 µT in counts
        
        point = apply_calibration(rawx, rawy, rawz, magcal)
        
        # After correction, should be close to (50, 0, 0)
        self.assertAlmostEqual(point.x, 50.0, places=1)
        self.assertAlmostEqual(point.y, 0.0, places=1)
        self.assertAlmostEqual(point.z, 0.0, places=1)

    def test_apply_calibration_with_soft_iron(self):
        """Test applying soft iron correction"""
        magcal = MagCalibration()
        magcal.V = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        # Scale X axis by 2
        magcal.invW = np.array([
            [2.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        
        rawx, rawy, rawz = 250, 0, 0  # 25 µT
        
        point = apply_calibration(rawx, rawy, rawz, magcal)
        
        # Should be scaled by 2
        self.assertAlmostEqual(point.x, 50.0, places=1)


class TestRawDataProcessing(unittest.TestCase):
    """Test raw data processing"""

    def setUp(self):
        """Set up test fixtures"""
        self.magcal = MagCalibration()

    def test_raw_data_reset(self):
        """Test raw data reset"""
        # Should not crash
        raw_data_reset(self.magcal, fusion_init)
        
        # Magcal should be reset
        self.assertEqual(self.magcal.FitError, 100.0)

    def test_add_magcal_data(self):
        """Test adding magnetometer data"""
        data = np.array([0, 0, 0, 0, 0, 0, 500, 0, 0], dtype=np.int16)
        
        add_magcal_data(data, self.magcal, quality_surface_gap_error)
        
        # Should have added one point
        count = np.sum(self.magcal.valid)
        self.assertEqual(count, 1)
        
        # Point should be stored
        self.assertEqual(self.magcal.BpFast[0, 0], 500)

    def test_choose_discard_with_good_coverage(self):
        """Test discarding when coverage is good (< 25% gaps)"""
        # Add many points (good coverage)
        for i in range(200):
            theta = (i / 200.0) * 2.0 * np.pi
            phi = np.arccos(1.0 - 2.0 * (i / 200.0))
            
            x = int(500 * np.sin(phi) * np.cos(theta))
            y = int(500 * np.sin(phi) * np.sin(theta))
            z = int(500 * np.cos(phi))
            
            self.magcal.BpFast[0, i] = x
            self.magcal.BpFast[1, i] = y
            self.magcal.BpFast[2, i] = z
            self.magcal.valid[i] = 1
        
        # Mock quality function that returns < 25%
        def mock_quality():
            return 20.0
        
        # Should discard outlier
        idx = choose_discard_magcal(self.magcal, mock_quality)
        
        self.assertGreaterEqual(idx, 0)
        self.assertLess(idx, 650)

    def test_is_float_ok(self):
        """Test float comparison"""
        # Identical values
        self.assertTrue(is_float_ok(1.0, 1.0))
        
        # Very close values
        self.assertTrue(is_float_ok(1.0, 1.00001))
        
        # Different values
        self.assertFalse(is_float_ok(1.0, 2.0))


class TestCalibrationConfirmation(unittest.TestCase):
    """Test calibration confirmation"""

    def test_cal1_data_confirmation(self):
        """Test Cal1 calibration echo"""
        cal_data_sent = np.zeros(19, dtype=np.float32)
        cal_data_sent[0:10] = [0, 0, 0, 0, 0, 0, 1.5, 2.5, 3.5, 50.0]
        
        cal_confirm_needed = [3]  # Expecting both Cal1 and Cal2
        confirmed = [False]
        
        def on_confirmed():
            confirmed[0] = True
        
        # Send matching Cal1 data
        data = np.array([0, 0, 0, 0, 0, 0, 1.5, 2.5, 3.5, 50.0], dtype=np.float32)
        cal1_data(data, cal_data_sent, cal_confirm_needed, on_confirmed)
        
        # Should clear bit 0
        self.assertEqual(cal_confirm_needed[0], 2)  # Only bit 1 left

    def test_cal2_data_confirmation(self):
        """Test Cal2 calibration echo"""
        cal_data_sent = np.zeros(19, dtype=np.float32)
        cal_data_sent[10:19] = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        
        cal_confirm_needed = [2]  # Expecting Cal2 only
        confirmed = [False]
        
        def on_confirmed():
            confirmed[0] = True
        
        # Send matching Cal2 data
        data = np.array([1, 0, 0, 0, 1, 0, 0, 0, 1], dtype=np.float32)
        cal2_data(data, cal_data_sent, cal_confirm_needed, on_confirmed)
        
        # Should clear bit 1 and call confirmed
        self.assertEqual(cal_confirm_needed[0], 0)
        self.assertTrue(confirmed[0])


if __name__ == '__main__':
    unittest.main()
