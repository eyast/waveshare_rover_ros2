"""
CRC16 calculation

Ports CRC16 algorithm from rawdata.c
Used for calibration packet checksum.
"""

import numpy as np


def crc16_byte(crc, data):
    """
    Calculate CRC16 with polynomial 0xA001 for single byte

    Ports: crc16 from rawdata.c

    Args:
        crc: current CRC value (uint16)
        data: single byte to add to CRC

    Returns:
        uint16: updated CRC value
    """
    crc = np.uint16(crc ^ data)

    for i in range(8):
        if crc & 1:
            crc = np.uint16((crc >> 1) ^ 0xA001)
        else:
            crc = np.uint16(crc >> 1)

    return crc


def crc16(data):
    """
    Calculate CRC16 for entire buffer

    Args:
        data: bytes or bytearray

    Returns:
        int: CRC value (0-65535)
    """
    crc = np.uint16(0)
    for byte in data:
        crc = crc16_byte(crc, byte)
    return int(crc)


# Alias for compatibility
def crc16_buffer(data):
    """Alias for crc16()"""
    return crc16(data)
