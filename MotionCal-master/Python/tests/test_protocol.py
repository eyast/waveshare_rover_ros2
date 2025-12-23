"""
Unit tests for serial protocol parser

Tests protocol.py ASCII message parsing
"""

import unittest
import numpy as np
from motioncal.serial.protocol import ProtocolParser, LineEnding


class TestProtocolParser(unittest.TestCase):
    """Test protocol parser"""

    def setUp(self):
        """Set up test fixtures"""
        self.parser = ProtocolParser()
        self.raw_data = None
        self.cal1_data = None
        self.cal2_data = None

        # Set up callbacks
        self.parser.on_raw_data = lambda data: setattr(self, 'raw_data', data)
        self.parser.on_cal1_data = lambda data: setattr(self, 'cal1_data', data)
        self.parser.on_cal2_data = lambda data: setattr(self, 'cal2_data', data)

    def test_parse_raw_message_lf(self):
        """Test parsing Raw: message with LF ending"""
        self.parser.set_line_ending(LineEnding.LF)
        message = b"Raw:100,200,300,10,20,30,50,60,70\n"
        
        self.parser.parse_bytes(message)
        
        self.assertIsNotNone(self.raw_data)
        self.assertEqual(len(self.raw_data), 9)
        self.assertEqual(self.raw_data[0], 100)  # ax
        self.assertEqual(self.raw_data[1], 200)  # ay
        self.assertEqual(self.raw_data[2], 300)  # az
        self.assertEqual(self.raw_data[6], 50)   # mx

    def test_parse_raw_message_crlf(self):
        """Test parsing Raw: message with CRLF ending"""
        self.parser.set_line_ending(LineEnding.CRLF)
        message = b"Raw:100,200,300,10,20,30,50,60,70\r\n"
        
        self.parser.parse_bytes(message)
        
        self.assertIsNotNone(self.raw_data)
        self.assertEqual(len(self.raw_data), 9)

    def test_parse_cal1_message(self):
        """Test parsing Cal1: message"""
        self.parser.set_line_ending(LineEnding.LF)
        message = b"Cal1:0.0,0.0,0.0,0.0,0.0,0.0,1.5,2.5,3.5,50.0\n"
        
        self.parser.parse_bytes(message)
        
        self.assertIsNotNone(self.cal1_data)
        self.assertEqual(len(self.cal1_data), 10)
        self.assertAlmostEqual(self.cal1_data[6], 1.5, places=5)   # mag offset x
        self.assertAlmostEqual(self.cal1_data[9], 50.0, places=5)  # field strength

    def test_parse_cal2_message(self):
        """Test parsing Cal2: message"""
        self.parser.set_line_ending(LineEnding.LF)
        message = b"Cal2:1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0\n"
        
        self.parser.parse_bytes(message)
        
        self.assertIsNotNone(self.cal2_data)
        self.assertEqual(len(self.cal2_data), 9)
        self.assertAlmostEqual(self.cal2_data[0], 1.0, places=5)
        self.assertAlmostEqual(self.cal2_data[4], 1.0, places=5)

    def test_parse_multiple_messages(self):
        """Test parsing multiple messages"""
        self.parser.set_line_ending(LineEnding.LF)
        messages = (
            b"Raw:100,200,300,10,20,30,50,60,70\n"
            b"Cal1:0.0,0.0,0.0,0.0,0.0,0.0,1.5,2.5,3.5,50.0\n"
        )
        
        self.parser.parse_bytes(messages)
        
        self.assertIsNotNone(self.raw_data)
        self.assertIsNotNone(self.cal1_data)

    def test_ignore_invalid_message(self):
        """Test that invalid messages are ignored"""
        self.parser.set_line_ending(LineEnding.LF)
        message = b"Invalid:message\n"
        
        self.parser.parse_bytes(message)
        
        # Should not crash, callbacks should not be called
        self.assertIsNone(self.raw_data)


if __name__ == '__main__':
    unittest.main()
