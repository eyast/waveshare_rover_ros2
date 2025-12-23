"""
ASCII protocol parser for MotionCal

Ports serialdata.parsing.c - State machine for parsing Raw:, Cal1:, Cal2: messages

The protocol supports three message types:
- Raw:ax,ay,az,gx,gy,gz,mx,my,mz\r - 9 int16 sensor values
- Cal1:f1,f2,...,f10\r - 10 float calibration echo (accel offset, gyro offset, mag offset, field)
- Cal2:f1,f2,...,f9\r - 9 float calibration echo (mag soft iron matrix)
"""

import numpy as np
from enum import IntEnum
from typing import Callable, Optional


class LineEnding(IntEnum):
    """Line ending modes"""
    NONE = 0  # No line ending
    LF = 1    # Line feed (\n)
    CR = 2    # Carriage return (\r)
    CRLF = 3  # Carriage return + line feed (\r\n)


class ParserState(IntEnum):
    """Parser state machine states"""
    STATE_WORD = 0   # Reading word (looking for Raw:, Cal1:, Cal2:)
    STATE_RAW = 1    # Reading Raw: message
    STATE_CAL1 = 2   # Reading Cal1: message
    STATE_CAL2 = 3   # Reading Cal2: message


class ProtocolParser:
    """
    ASCII protocol parser with state machine

    Ports: serialdata.parsing.c
    """

    def __init__(self):
        """Initialize protocol parser"""
        self.state = ParserState.STATE_WORD
        self.line_ending = LineEnding.LF

        # Current line buffer
        self.line_buffer = bytearray()
        self.max_line_length = 256

        # Callbacks
        self.on_raw_data: Optional[Callable[[np.ndarray], None]] = None
        self.on_cal1_data: Optional[Callable[[np.ndarray], None]] = None
        self.on_cal2_data: Optional[Callable[[np.ndarray], None]] = None

    def set_line_ending(self, ending: LineEnding):
        """
        Set line ending mode

        Args:
            ending: Line ending type
        """
        self.line_ending = ending

    def parse_byte(self, byte: int):
        """
        Parse single byte from serial stream

        Args:
            byte: Byte value (0-255)
        """
        # Check for line ending
        is_line_end = False

        if self.line_ending == LineEnding.NONE:
            # No line ending detection
            pass
        elif self.line_ending == LineEnding.LF:
            if byte == ord('\n'):
                is_line_end = True
        elif self.line_ending == LineEnding.CR:
            if byte == ord('\r'):
                is_line_end = True
        elif self.line_ending == LineEnding.CRLF:
            # For CRLF, we wait for \n and ignore preceding \r
            if byte == ord('\r'):
                return  # Skip CR
            elif byte == ord('\n'):
                is_line_end = True

        # Add to buffer
        if not is_line_end:
            if len(self.line_buffer) < self.max_line_length:
                self.line_buffer.append(byte)
            return

        # Process complete line
        self._process_line()
        self.line_buffer.clear()

    def parse_bytes(self, data: bytes):
        """
        Parse multiple bytes from serial stream

        Args:
            data: Bytes to parse
        """
        for byte in data:
            self.parse_byte(byte)

    def _process_line(self):
        """Process complete line buffer"""
        try:
            line = self.line_buffer.decode('ascii').strip()
        except UnicodeDecodeError:
            # Invalid ASCII, ignore line
            self.state = ParserState.STATE_WORD
            return

        if not line:
            return

        # Check for message type prefix
        if line.startswith('Raw:'):
            self._parse_raw_line(line[4:])
        elif line.startswith('Cal1:'):
            self._parse_cal1_line(line[5:])
        elif line.startswith('Cal2:'):
            self._parse_cal2_line(line[5:])
        else:
            # Unknown message, ignore
            pass

    def _parse_raw_line(self, data: str):
        """
        Parse Raw: message

        Ports: Raw: message parsing from serialdata.parsing.c

        Format: "ax,ay,az,gx,gy,gz,mx,my,mz"
        - 9 comma-separated int16 values
        - Accelerometer (ax, ay, az) in counts (±8192 = ±1g)
        - Gyroscope (gx, gy, gz) in counts (±2000 dps)
        - Magnetometer (mx, my, mz) in counts (0.1 µT per count)

        Args:
            data: Data portion after "Raw:"
        """
        try:
            # Split by comma
            parts = data.split(',')
            if len(parts) != 9:
                return

            # Parse as int16
            values = np.zeros(9, dtype=np.int16)
            for i, part in enumerate(parts):
                val = int(part.strip())
                # Clamp to int16 range
                if val < -32768:
                    val = -32768
                elif val > 32767:
                    val = 32767
                values[i] = val

            # Call callback
            if self.on_raw_data:
                self.on_raw_data(values)

        except (ValueError, IndexError):
            # Parse error, ignore
            pass

    def _parse_cal1_line(self, data: str):
        """
        Parse Cal1: calibration echo message

        Ports: Cal1: message parsing from serialdata.parsing.c

        Format: "f1,f2,f3,f4,f5,f6,f7,f8,f9,f10"
        - 10 comma-separated float values
        - Accel offset (3 floats, always 0.0 in ASCII mode)
        - Gyro offset (3 floats, always 0.0 in ASCII mode)
        - Mag offset (3 floats)
        - Mag field strength (1 float)

        Args:
            data: Data portion after "Cal1:"
        """
        try:
            # Split by comma
            parts = data.split(',')
            if len(parts) != 10:
                return

            # Parse as float32
            values = np.zeros(10, dtype=np.float32)
            for i, part in enumerate(parts):
                values[i] = float(part.strip())

            # Call callback
            if self.on_cal1_data:
                self.on_cal1_data(values)

        except (ValueError, IndexError):
            # Parse error, ignore
            pass

    def _parse_cal2_line(self, data: str):
        """
        Parse Cal2: calibration echo message

        Ports: Cal2: message parsing from serialdata.parsing.c

        Format: "f1,f2,f3,f4,f5,f6,f7,f8,f9"
        - 9 comma-separated float values
        - Mag soft iron matrix (9 floats, row-major 3x3 inverse matrix)

        Args:
            data: Data portion after "Cal2:"
        """
        try:
            # Split by comma
            parts = data.split(',')
            if len(parts) != 9:
                return

            # Parse as float32
            values = np.zeros(9, dtype=np.float32)
            for i, part in enumerate(parts):
                values[i] = float(part.strip())

            # Call callback
            if self.on_cal2_data:
                self.on_cal2_data(values)

        except (ValueError, IndexError):
            # Parse error, ignore
            pass

    def reset(self):
        """Reset parser state"""
        self.state = ParserState.STATE_WORD
        self.line_buffer.clear()
