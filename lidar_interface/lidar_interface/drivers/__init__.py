"""
LiDAR Driver Implementations
============================

Available drivers:
- YDLidarDriver: For YDLidar sensors (X4, etc.)

Add your own driver by implementing LidarBase.
"""

# Import available drivers
# Note: These may fail if SDK not installed - that's OK
try:
    from .ydlidar import YDLidarDriver
except ImportError:
    YDLidarDriver = None

__all__ = [
    "YDLidarDriver",
]
