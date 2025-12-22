"""
YDLidar Driver Implementation
=============================

Implements the LidarBase interface for YDLidar sensors.
Requires the ydlidar SDK to be installed.

Supported Models
----------------
- YDLidar X4 (tested)
- Other triangle ranging YDLidar models should work

Installation
------------
The YDLidar SDK must be installed separately.
See: https://github.com/YDLIDAR/YDLidar-SDK

Usage
-----
>>> from lidar_interface import LidarConfig
>>> from lidar_interface.drivers.ydlidar import YDLidarDriver
>>> 
>>> config = LidarConfig(port="/dev/ttyUSB0")
>>> with YDLidarDriver(config) as lidar:
...     scan = lidar.get_scan()
...     print(f"Got {len(scan.ranges)} points")
"""

import time
from typing import Optional

from ..base import LidarBase, LidarScan, LidarConfig


class YDLidarDriver(LidarBase):
    """
    YDLidar driver using the official SDK.
    
    This driver wraps the ydlidar Python bindings.
    Configuration is done through the LidarConfig dataclass.
    """
    
    def __init__(self, config: LidarConfig):
        """
        Initialize YDLidar driver.
        
        Parameters
        ----------
        config : LidarConfig
            Sensor configuration. Key fields:
            - port: Serial port (e.g., "/dev/ttyUSB0")
            - baudrate: Usually 128000 for X4
            - scan_frequency: Usually 10 Hz for X4
            - sample_rate: Usually 9 for X4
        """
        super().__init__(config)
        self._laser = None
        self._scan_buffer = None
        self._ydlidar = None
    
    def initialize(self) -> bool:
        """
        Initialize connection to YDLidar.
        
        Returns
        -------
        bool
            True if initialization successful
        """
        try:
            import ydlidar
            self._ydlidar = ydlidar
        except ImportError:
            raise ImportError(
                "YDLidar SDK not found. Please install it from "
                "https://github.com/YDLIDAR/YDLidar-SDK"
            )
        
        # Find port if auto-detect requested
        port = self.config.port
        if port == "auto":
            ports = ydlidar.lidarPortList()
            if not ports:
                return False
            port = list(ports.values())[0]
        
        # Create and configure lidar object
        self._laser = ydlidar.CYdLidar()
        self._laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        self._laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.config.baudrate)
        self._laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self._laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self._laser.setlidaropt(ydlidar.LidarPropScanFrequency, self.config.scan_frequency)
        self._laser.setlidaropt(ydlidar.LidarPropSampleRate, self.config.sample_rate)
        self._laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
        
        # Create scan buffer
        self._scan_buffer = ydlidar.LaserScan()
        
        # Initialize
        if not self._laser.initialize():
            return False
        
        self._initialized = True
        return True
    
    def start(self) -> bool:
        """
        Start scanning.
        
        Returns
        -------
        bool
            True if started successfully
        """
        if not self._initialized or self._laser is None:
            return False
        
        if not self._laser.turnOn():
            return False
        
        self._running = True
        return True
    
    def stop(self) -> bool:
        """
        Stop scanning.
        
        Returns
        -------
        bool
            True if stopped successfully
        """
        if self._laser is not None:
            self._laser.turnOff()
        self._running = False
        return True
    
    def get_scan(self) -> Optional[LidarScan]:
        """
        Get latest scan from the LiDAR.
        
        Returns
        -------
        LidarScan or None
            Latest scan data, or None if not available
        """
        if not self._running or self._laser is None:
            return None
        
        # Process scan from hardware
        if not self._laser.doProcessSimple(self._scan_buffer):
            return None
        
        # Convert to our format
        angles = []
        ranges = []
        intensities = []
        
        for point in self._scan_buffer.points:
            angles.append(point.angle)
            ranges.append(point.range)
            intensities.append(point.intensity)
        
        if len(angles) == 0:
            return None
        
        return LidarScan(
            timestamp=time.time(),
            angles=angles,
            ranges=ranges,
            intensities=intensities,
            range_min=self.config.range_min,
            range_max=self.config.range_max,
            frame_id=self.config.frame_id,
        )
    
    def shutdown(self) -> None:
        """Clean up and disconnect."""
        if self._laser is not None:
            self._laser.turnOff()
            self._laser.disconnecting()
            self._laser = None
        self._initialized = False
        self._running = False
