"""
Unit tests for calibration sender

Tests calibration_sender.py binary packet creation
"""

import unittest
import struct
import numpy as np
from motioncal.serial.calibration_sender import send_calibration
from motioncal.calibration.data_structures import MagCalibration
from motioncal.utils.crc import crc16


class TestCalibrationSender(unittest.TestCase):
    """Test calibration packet sender"""

    def setUp(self):
        """Set up test fixtures"""
        self.magcal = MagCalibration()
        self.cal_data_sent = np.zeros(19, dtype=np.float32)

    def test_packet_size(self):
        """Test that packet is 68 bytes"""
        packet = send_calibration(self.magcal, self.cal_data_sent)
        self.assertEqual(len(packet), 68)

    def test_packet_signature(self):
        """Test that packet has correct signature"""
        packet = send_calibration(self.magcal, self.cal_data_sent)
        
        self.assertEqual(packet[0], 117)
        self.assertEqual(packet[1], 84)

    def test_packet_crc(self):
        """Test that packet CRC is valid"""
        packet = send_calibration(self.magcal, self.cal_data_sent)
        
        # Extract CRC from end of packet
        crc_bytes = packet[-2:]
        packet_crc = struct.unpack('<H', crc_bytes)[0]
        
        # Calculate CRC of data (everything except CRC)
        calculated_crc = crc16(packet[:-2])
        
        self.assertEqual(packet_crc, calculated_crc)

    def test_packet_contains_mag_offset(self):
        """Test that packet contains magnetometer offset"""
        self.magcal.V = np.array([1.5, 2.5, 3.5], dtype=np.float32)
        
        packet = send_calibration(self.magcal, self.cal_data_sent)
        
        # Mag offset starts at byte 26 (2 signature + 6*4 zeros)
        offset_bytes = packet[26:38]
        offsets = struct.unpack('<fff', offset_bytes)
        
        self.assertAlmostEqual(offsets[0], 1.5, places=5)
        self.assertAlmostEqual(offsets[1], 2.5, places=5)
        self.assertAlmostEqual(offsets[2], 3.5, places=5)

    def test_packet_contains_field_strength(self):
        """Test that packet contains field strength"""
        self.magcal.B = 52.5
        
        packet = send_calibration(self.magcal, self.cal_data_sent)
        
        # Field strength at byte 38
        field_bytes = packet[38:42]
        field = struct.unpack('<f', field_bytes)[0]
        
        self.assertAlmostEqual(field, 52.5, places=5)

    def test_packet_contains_soft_iron(self):
        """Test that packet contains soft iron matrix"""
        self.magcal.invW = np.array([
            [1.1, 0.1, 0.2],
            [0.1, 0.9, 0.3],
            [0.2, 0.3, 1.0]
        ], dtype=np.float32)
        
        packet = send_calibration(self.magcal, self.cal_data_sent)
        
        # Soft iron starts at byte 42 (6 unique elements, 24 bytes)
        soft_iron_bytes = packet[42:66]
        soft_iron = struct.unpack('<ffffff', soft_iron_bytes)

        # Check diagonal and off-diagonal (6 values)
        self.assertAlmostEqual(soft_iron[0], 1.1, places=5)  # [0,0]
        self.assertAlmostEqual(soft_iron[1], 0.9, places=5)  # [1,1]
        self.assertAlmostEqual(soft_iron[2], 1.0, places=5)  # [2,2]
        self.assertAlmostEqual(soft_iron[3], 0.1, places=5)  # [0,1]
        self.assertAlmostEqual(soft_iron[4], 0.2, places=5)  # [0,2]
        self.assertAlmostEqual(soft_iron[5], 0.3, places=5)  # [1,2]

    def test_cal_data_sent_is_populated(self):
        """Test that cal_data_sent array is populated"""
        self.magcal.V = np.array([1.5, 2.5, 3.5], dtype=np.float32)
        self.magcal.B = 52.5
        
        packet = send_calibration(self.magcal, self.cal_data_sent)
        
        # Check that cal_data_sent has mag offset
        self.assertAlmostEqual(self.cal_data_sent[6], 1.5, places=5)
        self.assertAlmostEqual(self.cal_data_sent[7], 2.5, places=5)
        self.assertAlmostEqual(self.cal_data_sent[8], 3.5, places=5)
        
        # Check field strength
        self.assertAlmostEqual(self.cal_data_sent[9], 52.5, places=5)

    def test_accel_gyro_always_zero(self):
        """Test that accel and gyro offsets are always zero (ASCII mode)"""
        packet = send_calibration(self.magcal, self.cal_data_sent)
        
        # Accel offset at bytes 2-14
        accel_bytes = packet[2:14]
        accel = struct.unpack('<fff', accel_bytes)
        
        self.assertEqual(accel[0], 0.0)
        self.assertEqual(accel[1], 0.0)
        self.assertEqual(accel[2], 0.0)
        
        # Gyro offset at bytes 14-26
        gyro_bytes = packet[14:26]
        gyro = struct.unpack('<fff', gyro_bytes)
        
        self.assertEqual(gyro[0], 0.0)
        self.assertEqual(gyro[1], 0.0)
        self.assertEqual(gyro[2], 0.0)


if __name__ == '__main__':
    unittest.main()
