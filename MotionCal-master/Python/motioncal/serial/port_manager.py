"""
Serial port management for MotionCal

Handles port enumeration, opening, and reading/writing data.
"""

import serial
import serial.tools.list_ports
from typing import List, Optional, Tuple
from .protocol import LineEnding


def enumerate_ports() -> List[Tuple[str, str]]:
    """
    Enumerate available serial ports

    Returns:
        List of (port_name, description) tuples
        Example: [("/dev/ttyUSB0", "USB Serial"), ...]
    """
    ports = []
    for port_info in serial.tools.list_ports.comports():
        # port_info.device is the port name (e.g., "/dev/ttyUSB0", "COM3")
        # port_info.description is human-readable description
        ports.append((port_info.device, port_info.description))

    # Sort by port name
    ports.sort(key=lambda x: x[0])

    return ports


class PortManager:
    """
    Manages serial port connection

    Handles opening, closing, reading, and writing to serial ports.
    """

    # Standard baud rates supported by MotionCal
    BAUD_RATES = [
        300,
        1200,
        2400,
        4800,
        9600,
        19200,
        38400,
        57600,
        115200,
        230400,
    ]

    def __init__(self):
        """Initialize port manager"""
        self.port: Optional[serial.Serial] = None
        self.port_name: Optional[str] = None
        self.baud_rate: int = 115200
        self.line_ending: LineEnding = LineEnding.LF

    def is_open(self) -> bool:
        """
        Check if port is open

        Returns:
            True if port is open and ready
        """
        return self.port is not None and self.port.is_open

    def open(self, port_name: str, baud_rate: int = 115200,
             line_ending: LineEnding = LineEnding.LF) -> bool:
        """
        Open serial port

        Args:
            port_name: Port device name (e.g., "/dev/ttyUSB0", "COM3")
            baud_rate: Baud rate (default 115200)
            line_ending: Line ending mode (default LF)

        Returns:
            True if opened successfully, False otherwise
        """
        # Close existing port
        self.close()

        try:
            # Open port
            self.port = serial.Serial(
                port=port_name,
                baudrate=baud_rate,
                #bytesize=serial.EIGHTBITS,
                #parity=serial.PARITY_NONE,
                #stopbits=serial.STOPBITS_ONE,
                timeout=0.1,  # 10ms read timeout (non-blocking)
                # write_timeout=1.0,  # 1 second write timeout
            )

            self.port_name = port_name
            self.baud_rate = baud_rate
            self.line_ending = line_ending

            return True

        except (serial.SerialException, OSError) as e:
            # Failed to open port
            self.port = None
            self.port_name = None
            return False

    def close(self):
        """Close serial port"""
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
            self.port = None
            self.port_name = None

    def read_available(self) -> bytes:
        """
        Read all available data from port

        Returns:
            Bytes read (may be empty if no data available)
        """
        if not self.is_open():
            return b''

        try:
            # Read all available bytes
            waiting = self.port.in_waiting
            if waiting > 0:
                return self.port.read(waiting)
            return b''

        except (serial.SerialException, OSError):
            # Read error, close port
            self.close()
            return b''

    def write(self, data: bytes) -> bool:
        """
        Write data to port

        Args:
            data: Bytes to write

        Returns:
            True if written successfully, False otherwise
        """
        if not self.is_open():
            return False

        try:
            self.port.write(data)
            return True

        except (serial.SerialException, OSError):
            # Write error, close port
            self.close()
            return False

    def flush(self):
        """Flush input and output buffers"""
        if self.is_open():
            try:
                self.port.reset_input_buffer()
                self.port.reset_output_buffer()
            except (serial.SerialException, OSError):
                pass

    def __del__(self):
        """Cleanup on deletion"""
        self.close()
