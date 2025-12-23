"""
Quality metrics for magnetic calibration

Ports quality.c - Four quality indicators

Discussion of these metrics:
https://forum.pjrc.com/threads/59277-Motion-Sensor-Calibration-Tool-Parameter-Understanding
"""

import numpy as np
from ..utils.constants import MAGBUFFSIZE


# Module-level state (static variables in C)
class QualityState:
    """Container for quality computation state"""
    def __init__(self):
        self.count = 0
        self.spheredist = np.zeros(100, dtype=np.int32)  # Count per region
        self.spheredata = np.zeros((100, 3), dtype=np.float32)  # Sum of points per region
        self.sphereideal = np.zeros((100, 3), dtype=np.float32)  # Ideal sphere points
        self.sphereideal_initialized = False
        self.magnitude = np.zeros(MAGBUFFSIZE, dtype=np.float32)

        # Cached results
        self.quality_gaps_buffer = 0.0
        self.quality_variance_buffer = 0.0
        self.quality_wobble_buffer = 0.0
        self.quality_gaps_computed = False
        self.quality_variance_computed = False
        self.quality_wobble_computed = False


# Global state instance
_state = QualityState()


def sphere_region(x, y, z):
    """
    Return region index (0-99) on sphere of equal surface area

    The sphere is divided into 100 regions of equal area:
    - Arctic cap (1 region): lat > 78.52°
    - North temperate (15 regions): 42.84° < lat ≤ 78.52°
    - North tropic (34 regions): 0° < lat ≤ 42.84°
    - South tropic (34 regions): -42.84° ≤ lat < 0°
    - South temperate (15 regions): -78.52° ≤ lat < -42.84°
    - Antarctic cap (1 region): lat < -78.52°

    Ports: sphere_region from quality.c

    Args:
        x, y, z: Cartesian coordinates on sphere

    Returns:
        int: Region index 0-99
    """
    # Longitude: 0 to 2π (0 to 360 degrees)
    longitude = np.arctan2(y, x) + np.pi

    # Latitude: -π/2 to +π/2 (-90 to +90 degrees)
    latitude = (np.pi / 2.0) - np.arctan2(np.sqrt(x * x + y * y), z)

    # Sphere cap equations (see quality.c comments):
    # area of unit sphere = 4*pi
    # area of unit sphere cap = 2*pi*h  (h = cap height)
    # latitude of unit sphere cap = arcsin(1 - h)

    if latitude > 1.37046:  # 78.52 degrees
        # Arctic cap, 1 region
        region = 0
    elif latitude < -1.37046:  # -78.52 degrees
        # Antarctic cap, 1 region
        region = 99
    elif latitude > 0.74776 or latitude < -0.74776:  # ±42.84 degrees
        # Temperate zones, 15 regions each
        region = int(np.floor(longitude * (15.0 / (np.pi * 2.0))))
        if region < 0:
            region = 0
        elif region > 14:
            region = 14

        if latitude > 0.0:
            region += 1  # 1 to 15 (north temperate)
        else:
            region += 84  # 84 to 98 (south temperate)
    else:
        # Tropic zones, 34 regions each
        region = int(np.floor(longitude * (34.0 / (np.pi * 2.0))))
        if region < 0:
            region = 0
        elif region > 33:
            region = 33

        if latitude >= 0.0:
            region += 16  # 16 to 49 (north tropic)
        else:
            region += 50  # 50 to 83 (south tropic)

    return region


def quality_reset():
    """
    Reset quality computation state

    Ports: quality_reset from quality.c

    This must be called when calibration is reset.
    """
    global _state

    _state.count = 0
    _state.spheredist.fill(0)
    _state.spheredata.fill(0.0)

    # Initialize ideal sphere points (only once)
    if not _state.sphereideal_initialized:
        # Arctic cap (region 0)
        _state.sphereideal[0] = [0.0, 0.0, 1.0]

        # North temperate (regions 1-15)
        for i in range(1, 16):
            longitude = (float(i - 1) + 0.5) * (np.pi * 2.0 / 15.0)
            _state.sphereideal[i] = [
                np.cos(longitude) * np.cos(1.05911) * -1.0,
                np.sin(longitude) * np.cos(1.05911) * -1.0,
                np.sin(1.05911)
            ]

        # North tropic (regions 16-49)
        for i in range(16, 50):
            longitude = (float(i - 16) + 0.5) * (np.pi * 2.0 / 34.0)
            _state.sphereideal[i] = [
                np.cos(longitude) * np.cos(0.37388) * -1.0,
                np.sin(longitude) * np.cos(0.37388) * -1.0,
                np.sin(0.37388)
            ]

        # South tropic (regions 50-83)
        for i in range(50, 84):
            longitude = (float(i - 50) + 0.5) * (np.pi * 2.0 / 34.0)
            _state.sphereideal[i] = [
                np.cos(longitude) * np.cos(0.37388) * -1.0,
                np.sin(longitude) * np.cos(0.37388) * -1.0,
                np.sin(-0.37388)
            ]

        # South temperate (regions 84-98)
        for i in range(84, 99):
            longitude = (float(i - 1) + 0.5) * (np.pi * 2.0 / 15.0)
            _state.sphereideal[i] = [
                np.cos(longitude) * np.cos(1.05911) * -1.0,
                np.sin(longitude) * np.cos(1.05911) * -1.0,
                np.sin(-1.05911)
            ]

        # Antarctic cap (region 99)
        _state.sphereideal[99] = [0.0, 0.0, -1.0]

        _state.sphereideal_initialized = True

    # Invalidate cached results
    _state.quality_gaps_computed = False
    _state.quality_variance_computed = False
    _state.quality_wobble_computed = False


def quality_update(point):
    """
    Update quality metrics with a new calibrated point

    Ports: quality_update from quality.c

    Args:
        point: Point3D with x, y, z coordinates
    """
    global _state

    x, y, z = point.x, point.y, point.z

    # Store magnitude
    _state.magnitude[_state.count] = np.sqrt(x * x + y * y + z * z)

    # Determine sphere region and accumulate
    region = sphere_region(x, y, z)
    _state.spheredist[region] += 1
    _state.spheredata[region, 0] += x
    _state.spheredata[region, 1] += y
    _state.spheredata[region, 2] += z

    _state.count += 1

    # Invalidate cached results
    _state.quality_gaps_computed = False
    _state.quality_variance_computed = False
    _state.quality_wobble_computed = False


def quality_surface_gap_error():
    """
    Calculate surface gap error (0-100%)

    This metric measures how well the calibration data covers the sphere.
    Each of 100 regions gets a penalty:
    - 0 points: 1.0 error
    - 1 point: 0.2 error
    - 2 points: 0.01 error
    - 3+ points: 0.0 error

    Ports: quality_surface_gap_error from quality.c

    Returns:
        float: Gap error percentage (lower is better, goal: <15%)
    """
    global _state

    # Return cached result if available
    if _state.quality_gaps_computed:
        return _state.quality_gaps_buffer

    error = 0.0
    for i in range(100):
        num = _state.spheredist[i]
        if num == 0:
            error += 1.0
        elif num == 1:
            error += 0.2
        elif num == 2:
            error += 0.01
        # else: num >= 3, error += 0.0

    _state.quality_gaps_buffer = error
    _state.quality_gaps_computed = True
    return _state.quality_gaps_buffer


def quality_magnitude_variance_error():
    """
    Calculate magnitude variance error (0-100%)

    This metric measures how consistent the field magnitude is across
    all calibrated measurements. It's the standard deviation divided by
    the mean, expressed as a percentage.

    Ports: quality_magnitude_variance_error from quality.c

    Returns:
        float: Variance error percentage (lower is better, goal: <4.5%)
    """
    global _state

    # Return cached result if available
    if _state.quality_variance_computed:
        return _state.quality_variance_buffer

    if _state.count == 0:
        return 100.0

    # Calculate mean magnitude
    sum_mag = np.sum(_state.magnitude[:_state.count])
    mean = sum_mag / float(_state.count)

    # Calculate variance
    variance = 0.0
    for i in range(_state.count):
        diff = _state.magnitude[i] - mean
        variance += diff * diff
    variance /= float(_state.count)

    # Return as percentage of mean (coefficient of variation * 100)
    _state.quality_variance_buffer = np.sqrt(variance) / mean * 100.0
    _state.quality_variance_computed = True
    return _state.quality_variance_buffer


def quality_wobble_error():
    """
    Calculate wobble error (0-100%)

    This metric measures how much the regional averages deviate from
    an ideal sphere. It computes the average offset between actual
    regional centers and ideal regional centers.

    Ports: quality_wobble_error from quality.c

    Returns:
        float: Wobble error percentage (lower is better, goal: <4.0%)
    """
    global _state

    # Return cached result if available
    if _state.quality_wobble_computed:
        return _state.quality_wobble_buffer

    if _state.count == 0:
        return 100.0

    # Calculate mean radius
    sum_mag = np.sum(_state.magnitude[:_state.count])
    radius = sum_mag / float(_state.count)

    # Accumulate offsets from ideal positions
    xoff = 0.0
    yoff = 0.0
    zoff = 0.0
    n = 0

    for i in range(100):
        if _state.spheredist[i] > 0:
            # Average position for this region
            x = _state.spheredata[i, 0] / float(_state.spheredist[i])
            y = _state.spheredata[i, 1] / float(_state.spheredist[i])
            z = _state.spheredata[i, 2] / float(_state.spheredist[i])

            # Ideal position for this region (scaled to measured radius)
            xi = _state.sphereideal[i, 0] * radius
            yi = _state.sphereideal[i, 1] * radius
            zi = _state.sphereideal[i, 2] * radius

            # Accumulate offset
            xoff += x - xi
            yoff += y - yi
            zoff += z - zi
            n += 1

    if n == 0:
        return 100.0

    # Average offset
    xoff /= float(n)
    yoff /= float(n)
    zoff /= float(n)

    # Return as percentage of radius
    _state.quality_wobble_buffer = (
        np.sqrt(xoff * xoff + yoff * yoff + zoff * zoff) / radius * 100.0
    )
    _state.quality_wobble_computed = True
    return _state.quality_wobble_buffer


def quality_spherical_fit_error(magcal):
    """
    Return the spherical fit error from magnetic calibration

    This is Freescale's algorithm fit error, calculated during calibration.

    Ports: quality_spherical_fit_error from quality.c

    Args:
        magcal: MagCalibration instance

    Returns:
        float: Fit error percentage (lower is better, goal: <5.0%)
    """
    return float(magcal.FitError)
