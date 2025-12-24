#!/usr/bin/env python3
"""
Supporting tools for ROS nodes
"""

from typing import  Tuple, Union

from pyrover import IMUData_v2, ChassisInfo
from rover_msgs.msg import IMUv2, Temperature, Battery, Wheel


def to_ros_msg(ros_node, data: IMUData_v2) -> IMUv2:
    """Transforms a custom data type to the appropriapte
    ROS2 message type.
    
    Note that some functions return multiple instances.
    See: IMUData (returns IMU and Temperature)
    """
    try:
        ts = ros_node.get_clock().now().to_msg()
        header = ros_node.get_parameter('frame_id').value
        msg = IMUv2()
        msg.header.frame_id = header
        msg.header.stamp = ts
        msg.pitch = data.pitch
        msg.roll = data.roll
        msg.yaw = data.yaw
        msg.ax = data.ax
        msg.ay = data.ay
        msg.az = data.az
        msg.gx = data.gx
        msg.gy = data.gy
        msg.gz = data.gz
        msg.mx = data.mx
        msg.my = data.my
        msg.mz = data.mz
        return msg
    except Exception as e:
        ros_node.get_logger().debug(f"Transforming IMUv2 class to ros message failed:{e}")
    return IMUv2()