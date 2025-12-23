"""
Raw data processing and buffer management

Ports rawdata.c - Data flow orchestration
"""

import numpy as np
from ..utils.constants import (
    MAGBUFFSIZE,
    OVERSAMPLE_RATIO,
    G_PER_COUNT,
    DEG_PER_SEC_PER_COUNT
)
from ..data.sensor_data import AccelSensor, MagSensor, GyroSensor
from ..data.apply_calibration import apply_calibration
from ..calibration.magcal import magcal_run
from ..calibration.quality import quality_update


# Module-level state (static variables in C)
class RawDataState:
    """Container for raw data processing state"""
    def __init__(self):
        self.rawcount = OVERSAMPLE_RATIO
        self.force_orientation_counter = 0

        # Sensor accumulators
        self.accel = AccelSensor(
            Gp=np.zeros(3, dtype=np.float32),
            GpFast=np.zeros(3, dtype=np.float32)
        )
        self.mag = MagSensor(
            Bc=np.zeros(3, dtype=np.float32),
            BcFast=np.zeros(3, dtype=np.float32)
        )
        self.gyro = GyroSensor(
            Yp=np.zeros(3, dtype=np.float32),
            YpFast=np.zeros((OVERSAMPLE_RATIO, 3), dtype=np.float32)
        )


# Global state instance
_state = RawDataState()


def raw_data_reset(magcal, fusion_init_func):
    """
    Reset raw data processing state

    Ports: raw_data_reset from rawdata.c

    Args:
        magcal: MagCalibration instance
        fusion_init_func: Callback to initialize sensor fusion
    """
    global _state

    _state.rawcount = OVERSAMPLE_RATIO
    fusion_init_func()

    # Reset magcal
    magcal.reset()


def choose_discard_magcal(magcal, quality_surface_gap_error_func):
    """
    Choose which magnetometer reading to discard when buffer is full

    Ports: choose_discard_magcal from rawdata.c

    This is CRITICAL for good calibration. When the buffer is full,
    we must intelligently choose which old data to discard.

    Strategy:
    1. If gaps < 25%: Assume good coverage, discard point farthest from
       expected field strength
    2. Otherwise: Find 2 closest points, randomly discard one (spreads
       coverage by preferring diverse data)

    Args:
        magcal: MagCalibration instance
        quality_surface_gap_error_func: Callback to get gap error

    Returns:
        int: Index to discard (0 to MAGBUFFSIZE-1)
    """
    # Get current gap error
    gaps = quality_surface_gap_error_func()

    # Strategy 1: When coverage is good (gaps < 25%), discard outliers
    if gaps < 25.0:
        # Find point with magnitude farthest from expected field
        errormax = np.float32(0.0)
        discard_idx = 0

        for i in range(MAGBUFFSIZE):
            if magcal.valid[i]:
                # Apply calibration to get calibrated point
                rawx = magcal.BpFast[0, i]
                rawy = magcal.BpFast[1, i]
                rawz = magcal.BpFast[2, i]
                point = apply_calibration(rawx, rawy, rawz, magcal)

                # Calculate magnitude
                field = np.sqrt(point.x * point.x + point.y * point.y + point.z * point.z)

                # Error from expected field strength
                error = abs(field - magcal.B)

                if error > errormax:
                    errormax = error
                    discard_idx = i

        return discard_idx

    # Strategy 2: When coverage is poor, find closest pair and discard one
    minsum = np.uint64(0xFFFFFFFFFFFFFFFF)
    minindex = 0

    for i in range(MAGBUFFSIZE):
        if magcal.valid[i]:
            for j in range(i + 1, MAGBUFFSIZE):
                if magcal.valid[j]:
                    # Calculate squared distance between points
                    dx = np.int64(magcal.BpFast[0, i]) - np.int64(magcal.BpFast[0, j])
                    dy = np.int64(magcal.BpFast[1, i]) - np.int64(magcal.BpFast[1, j])
                    dz = np.int64(magcal.BpFast[2, i]) - np.int64(magcal.BpFast[2, j])
                    distsq = dx * dx + dy * dy + dz * dz

                    if distsq < minsum:
                        minsum = distsq
                        # Randomly choose i or j
                        minindex = i if (np.random.randint(0, 2) == 0) else j

    return minindex


def add_magcal_data(data, magcal, quality_surface_gap_error_func):
    """
    Add magnetometer reading to calibration buffer

    Ports: add_magcal_data from rawdata.c

    Args:
        data: int16 array of 9 values [ax,ay,az,gx,gy,gz,mx,my,mz]
        magcal: MagCalibration instance
        quality_surface_gap_error_func: Callback to get gap error
    """
    # Look for unused slot
    i = MAGBUFFSIZE
    for idx in range(MAGBUFFSIZE):
        if not magcal.valid[idx]:
            i = idx
            break

    # If buffer is full, choose which data to discard
    if i >= MAGBUFFSIZE:
        i = choose_discard_magcal(magcal, quality_surface_gap_error_func)
        if i < 0 or i >= MAGBUFFSIZE:
            i = np.random.randint(0, MAGBUFFSIZE)

    # Add to calibration buffer (magnetometer is data[6:9])
    magcal.BpFast[0, i] = data[6]
    magcal.BpFast[1, i] = data[7]
    magcal.BpFast[2, i] = data[8]
    magcal.valid[i] = 1


def is_float_ok(actual, expected):
    """
    Check if two floats are close enough for calibration confirmation

    Ports: is_float_ok from rawdata.c

    Args:
        actual: Received value
        expected: Expected value

    Returns:
        bool: True if close enough
    """
    err = abs(actual - expected)
    maxerr = np.float32(0.0001) + abs(expected) * np.float32(0.00003)
    return err <= maxerr


def cal1_data(data, cal_data_sent, cal_confirm_needed_ref, calibration_confirmed_func):
    """
    Process Cal1 calibration echo message

    Ports: cal1_data from rawdata.c

    Args:
        data: Array of 10 floats from Cal1 message
        cal_data_sent: Array of sent calibration data
        cal_confirm_needed_ref: Reference to cal_confirm_needed (list with [value])
        calibration_confirmed_func: Callback when calibration confirmed
    """
    if cal_confirm_needed_ref[0]:
        # Check if all 10 values match what we sent
        ok = True
        for i in range(10):
            if not is_float_ok(data[i], cal_data_sent[i]):
                ok = False
                break

        if ok:
            # Clear bit 0 (got cal1 confirm)
            cal_confirm_needed_ref[0] &= ~1
            if cal_confirm_needed_ref[0] == 0:
                calibration_confirmed_func()


def cal2_data(data, cal_data_sent, cal_confirm_needed_ref, calibration_confirmed_func):
    """
    Process Cal2 calibration echo message

    Ports: cal2_data from rawdata.c

    Args:
        data: Array of 9 floats from Cal2 message
        cal_data_sent: Array of sent calibration data
        cal_confirm_needed_ref: Reference to cal_confirm_needed (list with [value])
        calibration_confirmed_func: Callback when calibration confirmed
    """
    if cal_confirm_needed_ref[0]:
        # Check if all 9 values match what we sent (offset by 10)
        ok = True
        for i in range(9):
            if not is_float_ok(data[i], cal_data_sent[i + 10]):
                ok = False
                break

        if ok:
            # Clear bit 1 (got cal2 confirm)
            cal_confirm_needed_ref[0] &= ~2
            if cal_confirm_needed_ref[0] == 0:
                calibration_confirmed_func()


def raw_data(data, magcal, fusion_init_func, fusion_update_func,
             fusion_read_func, quality_surface_gap_error_func):
    """
    Process raw IMU data

    Ports: raw_data from rawdata.c

    This is the main data processing function. It:
    1. Adds magnetometer data to calibration buffer
    2. Runs magnetic calibration periodically
    3. Accumulates sensor readings for oversampling
    4. Calls sensor fusion when OVERSAMPLE_RATIO readings collected
    5. Updates quality metrics

    Args:
        data: int16 array of 9 values [ax,ay,az,gx,gy,gz,mx,my,mz]
        magcal: MagCalibration instance
        fusion_init_func: Callback to initialize sensor fusion
        fusion_update_func: Callback to update sensor fusion
        fusion_read_func: Callback to read current orientation
        quality_surface_gap_error_func: Callback to get gap error

    Returns:
        Quaternion or None: Current orientation if fusion updated, else None
    """
    global _state

    # Add to calibration buffer
    add_magcal_data(data, magcal, quality_surface_gap_error_func)

    # Store previous hard iron offset
    old_V = magcal.V.copy()

    # Run calibration (returns 1 if new calibration accepted)
    if magcal_run(magcal):
        # Calculate change in hard iron offset
        dx = old_V[0] - magcal.V[0]
        dy = old_V[1] - magcal.V[1]
        dz = old_V[2] - magcal.V[2]
        magdiff = np.sqrt(dx * dx + dy * dy + dz * dz)

        # If hard iron changed significantly, reset fusion
        if magdiff > 0.8:
            fusion_init_func()
            _state.rawcount = OVERSAMPLE_RATIO
            _state.force_orientation_counter = 240

    # Handle delayed fusion reset
    if _state.force_orientation_counter > 0:
        _state.force_orientation_counter -= 1
        if _state.force_orientation_counter == 0:
            fusion_init_func()
            _state.rawcount = OVERSAMPLE_RATIO

    # Start new accumulation cycle if needed
    if _state.rawcount >= OVERSAMPLE_RATIO:
        _state.accel.Gp.fill(0.0)
        _state.mag.Bc.fill(0.0)
        _state.gyro.Yp.fill(0.0)
        _state.rawcount = 0

    # Accumulate accelerometer (data[0:3])
    x = np.float32(data[0]) * G_PER_COUNT
    y = np.float32(data[1]) * G_PER_COUNT
    z = np.float32(data[2]) * G_PER_COUNT
    _state.accel.GpFast[0] = x
    _state.accel.GpFast[1] = y
    _state.accel.GpFast[2] = z
    _state.accel.Gp[0] += x
    _state.accel.Gp[1] += y
    _state.accel.Gp[2] += z

    # Accumulate gyroscope (data[3:6])
    x = np.float32(data[3]) * DEG_PER_SEC_PER_COUNT
    y = np.float32(data[4]) * DEG_PER_SEC_PER_COUNT
    z = np.float32(data[5]) * DEG_PER_SEC_PER_COUNT
    _state.gyro.Yp[0] += x
    _state.gyro.Yp[1] += y
    _state.gyro.Yp[2] += z
    _state.gyro.YpFast[_state.rawcount, 0] = x
    _state.gyro.YpFast[_state.rawcount, 1] = y
    _state.gyro.YpFast[_state.rawcount, 2] = z

    # Apply calibration to magnetometer (data[6:9])
    point = apply_calibration(data[6], data[7], data[8], magcal)
    _state.mag.BcFast[0] = point.x
    _state.mag.BcFast[1] = point.y
    _state.mag.BcFast[2] = point.z
    _state.mag.Bc[0] += point.x
    _state.mag.Bc[1] += point.y
    _state.mag.Bc[2] += point.z

    # Update quality metrics with calibrated point
    quality_update(point)

    _state.rawcount += 1

    # When oversampling complete, average and update fusion
    current_orientation = None
    if _state.rawcount >= OVERSAMPLE_RATIO:
        ratio = np.float32(1.0) / np.float32(OVERSAMPLE_RATIO)
        _state.accel.Gp *= ratio
        _state.gyro.Yp *= ratio
        _state.mag.Bc *= ratio

        # Update sensor fusion
        fusion_update_func(_state.accel, _state.mag, _state.gyro, magcal)
        current_orientation = fusion_read_func()

    return current_orientation
