"""
Calibration packet sender

Ports send_calibration() from rawdata.c
Builds and sends 68-byte binary calibration packet to device.
"""

import struct
import numpy as np
from ..utils.crc import crc16_buffer
from ..utils.constants import CAL_SIGNATURE_BYTE1, CAL_SIGNATURE_BYTE2


# Global calibration confirmation tracking
cal_data_sent = np.zeros(19, dtype=np.float32)
cal_confirm_needed = 0


def copy_lsb_first(buf, offset, value):
    """
    Copy float32 value to buffer in LSB-first (little-endian) order

    Ports: copy_lsb_first from rawdata.c

    Args:
        buf: bytearray to write to
        offset: starting position in buffer
        value: float32 value to write

    Returns:
        int: next offset (offset + 4)
    """
    # Pack float as little-endian 32-bit float
    bytes_val = struct.pack('<f', np.float32(value))
    buf[offset:offset + 4] = bytes_val
    return offset + 4


def build_calibration_packet(magcal, cal_data_sent):
    """
    Build 68-byte calibration packet

    Ports: send_calibration from rawdata.c

    Packet structure:
        Bytes 0-1:   Signature (117, 84)
        Bytes 2-13:  Accelerometer offsets (3 × float32, all zeros)
        Bytes 14-25: Gyroscope offsets (3 × float32, all zeros)
        Bytes 26-37: Magnetometer hard iron offsets (3 × float32)
        Bytes 38-41: Magnetic field strength (1 × float32)
        Bytes 42-65: Soft iron matrix (6 × float32, symmetric elements)
        Bytes 66-67: CRC16 checksum

    Args:
        magcal: MagCalibration instance
        cal_data_sent: np.ndarray(19) to store sent calibration data

    Returns:
        bytearray: 68-byte packet ready to send
    """
    buf = bytearray(68)
    offset = 0

    # Signature
    buf[offset] = CAL_SIGNATURE_BYTE1  # 117 = 'u'
    buf[offset + 1] = CAL_SIGNATURE_BYTE2  # 84 = 'T'
    offset += 2

    # Accelerometer offsets (3 × float32, all zeros)
    for i in range(3):
        offset = copy_lsb_first(buf, offset, 0.0)
        cal_data_sent[0 + i] = np.float32(0.0)

    # Gyroscope offsets (3 × float32, all zeros)
    for i in range(3):
        offset = copy_lsb_first(buf, offset, 0.0)
        cal_data_sent[3 + i] = np.float32(0.0)

    # Magnetometer hard iron offsets (3 × float32)
    for i in range(3):
        offset = copy_lsb_first(buf, offset, magcal.V[i])
        cal_data_sent[6 + i] = magcal.V[i]

    # Magnetic field strength
    offset = copy_lsb_first(buf, offset, magcal.B)

    # Soft iron matrix (6 unique elements of symmetric 3×3 matrix)
    # Diagonal elements
    offset = copy_lsb_first(buf, offset, magcal.invW[0, 0])  # [0][0]
    offset = copy_lsb_first(buf, offset, magcal.invW[1, 1])  # [1][1]
    offset = copy_lsb_first(buf, offset, magcal.invW[2, 2])  # [2][2]
    # Off-diagonal elements
    offset = copy_lsb_first(buf, offset, magcal.invW[0, 1])  # [0][1]
    offset = copy_lsb_first(buf, offset, magcal.invW[0, 2])  # [0][2]
    offset = copy_lsb_first(buf, offset, magcal.invW[1, 2])  # [1][2]

    # Store sent calibration data for verification
    cal_data_sent[9] = magcal.B
    cal_data_sent[10] = magcal.invW[0, 0]
    cal_data_sent[11] = magcal.invW[0, 1]
    cal_data_sent[12] = magcal.invW[0, 2]
    cal_data_sent[13] = magcal.invW[1, 0]
    cal_data_sent[14] = magcal.invW[1, 1]
    cal_data_sent[15] = magcal.invW[1, 2]
    cal_data_sent[16] = magcal.invW[2, 0]
    cal_data_sent[17] = magcal.invW[2, 1]
    cal_data_sent[18] = magcal.invW[2, 2]

    # Calculate CRC16 of first 66 bytes
    crc = crc16_buffer(buf[:66])

    # Append CRC16 in little-endian order
    buf[66] = crc & 0xFF
    buf[67] = (crc >> 8) & 0xFF

    return buf


def send_calibration(magcal, cal_data_sent):
    """
    Build calibration packet for sending to device

    Args:
        magcal: MagCalibration instance with calibration data
        cal_data_sent: np.ndarray(19) to store sent calibration data

    Returns:
        bytes: 68-byte calibration packet
    """
    packet = build_calibration_packet(magcal, cal_data_sent)
    return packet
