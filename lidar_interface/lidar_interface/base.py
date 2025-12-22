"""
Abstract LiDAR Interface
========================

This module provides an abstract interface for LiDAR sensors.
Implement this interface to add support for new LiDAR types.

The interface is designed to be:
1. ROS2-compatible but not ROS2-dependent
2. Easy to implement for various LiDAR types
3. Suitable for both 2D and simple 3D LiDAR

Example Implementation
----------------------
>>> class MyLidar(LidarBase):
...     def initialize(self) -> bool:
...         # Connect to hardware
...         self._device = MyLidarSDK(port=self.port)
...         return self._device.connect()
...     
...     def start(self) -> bool:
...         return self._device.start_scanning()
...     
...     def stop(self) -> bool:
...         return self._device.stop_scanning()
...     
...     def get_scan(self) -> Optional[LidarScan]:
...         raw = self._device.get_scan()
...         if raw is None:
...             return None
...         return LidarScan(
...             timestamp=time.time(),
...             angles=raw.angles,
...             ranges=raw.distances,
...         )
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, List
import time


@dataclass
class LidarScan:
    """
    A single LiDAR scan.
    
    Attributes
    ----------
    timestamp : float
        Time when scan was taken (seconds since epoch)
    angles : List[float]
        Angle for each measurement (radians)
    ranges : List[float]
        Distance for each measurement (meters)
    intensities : List[float], optional
        Intensity/reflectivity for each measurement
    angle_min : float
        Minimum angle in scan (radians)
    angle_max : float
        Maximum angle in scan (radians)
    angle_increment : float
        Angle between measurements (radians)
    range_min : float
        Minimum valid range (meters)
    range_max : float
        Maximum valid range (meters)
    frame_id : str
        TF frame ID for this scan
    """
    timestamp: float
    angles: List[float]
    ranges: List[float]
    intensities: List[float] = field(default_factory=list)
    angle_min: float = 0.0
    angle_max: float = 0.0
    angle_increment: float = 0.0
    range_min: float = 0.1
    range_max: float = 12.0
    frame_id: str = "laser"
    
    def __post_init__(self):
        """Compute angle bounds if not set."""
        if self.angles and self.angle_min == 0.0 and self.angle_max == 0.0:
            self.angle_min = min(self.angles)
            self.angle_max = max(self.angles)
            if len(self.angles) > 1:
                self.angle_increment = (self.angle_max - self.angle_min) / (len(self.angles) - 1)


@dataclass
class LidarConfig:
    """
    Configuration for a LiDAR sensor.
    
    Attributes
    ----------
    port : str
        Serial port or device path
    baudrate : int
        Serial baudrate (if applicable)
    scan_frequency : float
        Desired scan frequency in Hz
    sample_rate : int
        Samples per second (if configurable)
    frame_id : str
        TF frame ID for published scans
    range_min : float
        Minimum valid range in meters
    range_max : float
        Maximum valid range in meters
    """
    port: str = "/dev/ttyUSB0"
    baudrate: int = 128000
    scan_frequency: float = 10.0
    sample_rate: int = 9
    frame_id: str = "laser"
    range_min: float = 0.1
    range_max: float = 12.0


class LidarBase(ABC):
    """
    Abstract base class for LiDAR sensors.
    
    Implement this class to add support for a new LiDAR type.
    The minimum implementation requires:
    - initialize(): Connect to hardware
    - start(): Begin scanning
    - stop(): Stop scanning
    - get_scan(): Return latest scan data
    - shutdown(): Clean up resources
    """
    
    def __init__(self, config: LidarConfig):
        """
        Initialize with configuration.
        
        Parameters
        ----------
        config : LidarConfig
            Sensor configuration
        """
        self.config = config
        self._initialized = False
        self._running = False
    
    @property
    def is_initialized(self) -> bool:
        """Check if sensor is initialized."""
        return self._initialized
    
    @property
    def is_running(self) -> bool:
        """Check if sensor is actively scanning."""
        return self._running
    
    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize connection to LiDAR hardware.
        
        Returns
        -------
        bool
            True if initialization successful
        """
        pass
    
    @abstractmethod
    def start(self) -> bool:
        """
        Start LiDAR scanning.
        
        Returns
        -------
        bool
            True if started successfully
        """
        pass
    
    @abstractmethod
    def stop(self) -> bool:
        """
        Stop LiDAR scanning.
        
        Returns
        -------
        bool
            True if stopped successfully
        """
        pass
    
    @abstractmethod
    def get_scan(self) -> Optional[LidarScan]:
        """
        Get the latest scan data.
        
        Returns
        -------
        LidarScan or None
            Latest scan, or None if no scan available
        """
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        """
        Clean up resources and disconnect.
        """
        pass
    
    def __enter__(self):
        """Context manager entry."""
        self.initialize()
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
        self.shutdown()


class DummyLidar(LidarBase):
    """
    Dummy LiDAR for testing without hardware.
    
    Generates fake scan data in a circle pattern.
    """
    
    def __init__(self, config: LidarConfig):
        super().__init__(config)
        import math
        self._math = math
        self._time_offset = 0.0
    
    def initialize(self) -> bool:
        self._initialized = True
        return True
    
    def start(self) -> bool:
        self._running = True
        self._time_offset = time.time()
        return True
    
    def stop(self) -> bool:
        self._running = False
        return True
    
    def get_scan(self) -> Optional[LidarScan]:
        if not self._running:
            return None
        
        # Generate fake scan: 360 points in a circle with some noise
        import random
        num_points = 360
        angles = []
        ranges = []
        intensities = []
        
        t = time.time() - self._time_offset
        
        for i in range(num_points):
            angle = self._math.radians(i)
            # Simulate a room with walls at ~3m, with some variation
            base_range = 3.0 + 0.5 * self._math.sin(4 * angle + t)
            range_val = base_range + random.uniform(-0.1, 0.1)
            
            angles.append(angle)
            ranges.append(max(self.config.range_min, min(self.config.range_max, range_val)))
            intensities.append(100.0)
        
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
        self._initialized = False
        self._running = False
