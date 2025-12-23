"""
Serial communication for MotionCal

Handles serial port enumeration, ASCII protocol parsing, and data transmission.
"""

from .protocol import ProtocolParser, LineEnding
from .port_manager import PortManager, enumerate_ports

__all__ = [
    'ProtocolParser',
    'LineEnding',
    'PortManager',
    'enumerate_ports',
]
