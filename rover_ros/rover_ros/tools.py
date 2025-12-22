#!/usr/bin/env python3
"""
Supporting tools for ROS nodes
"""

from typing import  Tuple, Union

from pyrover import IMUData, ChassisInfo
from rover_msgs.msg import IMU, Temperature, Battery, Wheel


def to_ros_msg(ros_node, data: Union[IMUData, ChassisInfo, float]) -> Union[Tuple[IMU, Temperature], Battery, None]:
    """Transforms a custom data type to the appropriapte
    ROS2 message type.
    
    Note that some functions return multiple instances.
    See: IMUData (returns IMU and Temperature)
    """
    try:
        ts = ros_node.get_clock().now().to_msg()
        header = ros_node.get_parameter('frame_id').value
        if isinstance(data, IMUData):
            msg = IMU()
            msg.header.frame_id = header
            msg.header.stamp = ts
            msg.pitch = data.pitch
            msg.roll = data.roll
            msg.yaw = data.yaw
            msg.axx = data.accel_x
            msg.axy = data.accel_y
            msg.axz = data.accel_z
            msg.gx = data.mag_x
            msg.gy = data.mag_y
            msg.gz = data.mag_z
            temp = Temperature()
            temp.header.frame_id = header
            temp.header.stamp = ts
            temp.value = data.temperature
            return msg, temp
        elif isinstance(data, ChassisInfo):
            voltage: float = data.voltage if data else 0.0
            pct = ros_node.battery_estimator.voltage_to_soc(voltage) * 100 # type : ignore
            l: float = data.left_speed if data else 0.0
            r: float = data.right_speed if data else 0.0
            battery_msg = Battery()
            battery_msg.header.frame_id = header
            battery_msg.header.stamp = ts
            battery_msg.value = pct
            wheel_msg = Wheel()
            battery_msg.header.frame_id = header
            battery_msg.header.stamp = ts
            wheel_msg.l = l
            wheel_msg.r = r
            return battery_msg, wheel_msg
    except Exception as e:
        ros_node.get_logger().debug(f"Transforming data failed:{e}")