"""
Unit tests for CRC calculation

Tests crc.py CRC16 implementation
"""

import unittest
from motioncal.utils.crc import crc16


class TestCRC16(unittest.TestCase):
    """Test CRC16 calculation"""

    def test_crc16_empty(self):
        """Test CRC of empty data"""
        data = b''
        crc = crc16(data)
        self.assertEqual(crc, 0)

    def test_crc16_single_byte(self):
        """Test CRC of single byte"""
        data = b'\x00'
        crc = crc16(data)
        self.assertEqual(crc, 0)
        
        data = b'\xFF'
        crc = crc16(data)
        self.assertNotEqual(crc, 0)

    def test_crc16_known_values(self):
        """Test CRC with known test vectors"""
        # Test vector: "123456789" should give specific CRC
        data = b'123456789'
        crc = crc16(data)
        # CRC-16 with polynomial 0xA001 should give 0xBB3D
        self.assertEqual(crc, 0xBB3D)

    def test_crc16_different_data(self):
        """Test that different data gives different CRC"""
        data1 = b'hello'
        data2 = b'world'
        
        crc1 = crc16(data1)
        crc2 = crc16(data2)
        
        self.assertNotEqual(crc1, crc2)

    def test_crc16_same_data(self):
        """Test that same data gives same CRC"""
        data = b'test data'
        
        crc1 = crc16(data)
        crc2 = crc16(data)
        
        self.assertEqual(crc1, crc2)

    def test_crc16_detects_single_bit_error(self):
        """Test that CRC detects single bit errors"""
        data1 = b'\x00\x00\x00\x00'
        data2 = b'\x01\x00\x00\x00'
        
        crc1 = crc16(data1)
        crc2 = crc16(data2)
        
        self.assertNotEqual(crc1, crc2)

    def test_crc16_calibration_packet(self):
        """Test CRC on calibration packet format"""
        # Simulate 66-byte calibration data (signature + floats)
        import struct
        
        data = bytearray()
        data.extend(struct.pack('<BB', 117, 84))  # Signature
        
        # Add 16 floats (accel, gyro, mag offset, field, soft iron)
        for i in range(16):
            data.extend(struct.pack('<f', float(i)))
        
        crc = crc16(bytes(data))
        
        # Should be a valid 16-bit value
        self.assertGreaterEqual(crc, 0)
        self.assertLessEqual(crc, 0xFFFF)


if __name__ == '__main__':
    unittest.main()
