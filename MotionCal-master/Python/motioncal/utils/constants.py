"""
Constants from imuread.h

All constants, thresholds, and scaling factors used throughout MotionCal.
These values are preserved exactly from the C++ implementation.
"""

import numpy as np

# Buffer sizes
MAGBUFFSIZE = 650  # Freescale's lib needs at least 392
BUFFER_SIZE = 512
TIMEOUT_MSEC = 14

# Sensor sampling
SENSORFS = 100  # Sensor sample rate (Hz)
OVERSAMPLE_RATIO = 4

# Scale factors from imuread.h
G_PER_COUNT = np.float32(0.0001220703125)  # 1/8192 - accelerometer
UT_PER_COUNT = np.float32(0.1)  # magnetometer (µT per count)
DEG_PER_SEC_PER_COUNT = np.float32(0.0625)  # 1/16 - gyroscope

# Magnetic calibration constants (from magcal.c)
FXOS8700_UTPERCOUNT = np.float32(0.1)
DEFAULTB = np.float32(50.0)  # default geomagnetic field (µT)
ONETHIRD = np.float32(0.33333333)
ONESIXTH = np.float32(0.166666667)

# Minimum measurements for calibration algorithms
MINMEASUREMENTS4CAL = 40  # 4 element matrix inversion
MINMEASUREMENTS7CAL = 100  # 7 element eigendecomposition
MINMEASUREMENTS10CAL = 150  # 10 element eigendecomposition

# Geomagnetic field validation range (µT)
MINBFITUT = np.float32(22.0)  # minimum valid field strength
MAXBFITUT = np.float32(67.0)  # maximum valid field strength

# Fit error aging
FITERRORAGINGSECS = np.float32(7200.0)  # 2 hours: time for fit error to age by e=2.718

# Quality thresholds for "Send Calibration" button
QUALITY_GAPS_THRESHOLD = 15.0  # surface coverage (%)
QUALITY_VARIANCE_THRESHOLD = 4.5  # magnitude consistency (%)
QUALITY_WOBBLE_THRESHOLD = 4.0  # sphericity (%)
QUALITY_FIT_THRESHOLD = 5.0  # fit error (%)

# Line ending types
LINE_ENDING_NOTSET = 0
LINE_ENDING_LF = 1  # "\n"
LINE_ENDING_CR = 2  # "\r"
LINE_ENDING_CRLF = 3  # "\r\n"

# Mahony AHRS constants (from mahony.c)
MAHONY_TWO_KP = np.float32(0.04)  # 2 * proportional gain (2.0 * 0.02)
MAHONY_TWO_KI = np.float32(0.0)  # 2 * integral gain (disabled)
MAHONY_INV_SAMPLE_RATE = np.float32(0.01)  # 1.0 / SENSORFS

# Calibration packet signature
CAL_SIGNATURE_BYTE1 = 117  # 'u'
CAL_SIGNATURE_BYTE2 = 84  # 'T'

# CRC16 polynomial
CRC16_POLY = 0xA001
