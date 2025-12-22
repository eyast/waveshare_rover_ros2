"""
LiDAR Interface Library
=======================

A hardware-agnostic LiDAR abstraction layer for robotics.

This library provides:
- Abstract base class for LiDAR drivers (LidarBase)
- Common data types (LidarScan, LidarConfig)
- Concrete implementations in the drivers subpackage

Usage
-----
>>> from lidar_interface import LidarConfig, LidarScan, LidarBase
>>> from lidar_interface.drivers import YDLidarDriver
>>>
>>> config = LidarConfig(port="/dev/ttyUSB0")
>>> with YDLidarDriver(config) as lidar:
...     scan = lidar.get_scan()

Implementing Custom Drivers
---------------------------
To support a new LiDAR sensor, subclass LidarBase:

>>> class MyLidarDriver(LidarBase):
...     def initialize(self) -> bool:
...         # Connect to hardware
...         pass
...     def start(self) -> bool:
...         # Begin scanning
...         pass
...     def stop(self) -> bool:
...         # Stop scanning
...         pass
...     def get_scan(self) -> Optional[LidarScan]:
...         # Return latest scan
...         pass
...     def shutdown(self) -> None:
...         # Clean up
...         pass
"""

from .base import LidarBase, LidarScan, LidarConfig, DummyLidar

__all__ = [
    "LidarBase",
    "LidarScan",
    "LidarConfig",
    "DummyLidar",
]

__version__ = "0.1.0"
