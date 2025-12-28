#!/usr/bin/env python3
"""
Supporting tools for ROS2 nodes (New Line-Based Protocol)
=========================================================

Provides utilities for converting between pyrover data types
and ROS2 messages.
"""

import math
from typing import Tuple

from pyrover import IMUData, PowerData
from rover_msgs.msg import IMUv2, Temperature, Battery


def imu_to_ros_msg(ros_node, data: IMUData) -> IMUv2:
    """
    Convert IMUData to ROS2 IMUv2 message.
    
    Args:
        ros_node: ROS2 node (for clock and parameters)
        data: IMUData from pyrover
        
    Returns:
        IMUv2 ROS2 message
    """
    try:
        ts = ros_node.get_clock().now().to_msg()
        frame_id = ros_node.get_parameter('frame_id').value
        
        msg = IMUv2()
        msg.header.frame_id = frame_id
        msg.header.stamp = ts
        msg.y = data.yaw
        msg.p = data.pitch
        msg.r = data.roll
        msg.t = data.temp
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
        ros_node.get_logger().debug(f"IMU conversion failed: {e}")
        return IMUv2()


def temp_to_ros_msg(ros_node, temp: float) -> Temperature:
    """
    Convert temperature value to ROS2 Temperature message.
    
    Args:
        ros_node: ROS2 node
        temp: Temperature in Celsius
        
    Returns:
        Temperature ROS2 message
    """
    msg = Temperature()
    msg.header.stamp = ros_node.get_clock().now().to_msg()
    msg.header.frame_id = ros_node.get_parameter('frame_id').value
    msg.value = temp
    return msg


def power_to_battery_msg(ros_node, data: PowerData, estimator=None) -> Battery:
    """
    Convert PowerData to ROS2 Battery message.
    
    Args:
        ros_node: ROS2 node
        data: PowerData from pyrover
        estimator: Optional BatteryEstimator for SoC calculation
        
    Returns:
        Battery ROS2 message with percentage
    """
    msg = Battery()
    msg.header.stamp = ros_node.get_clock().now().to_msg()
    msg.header.frame_id = ros_node.get_parameter('frame_id').value
    
    if estimator:
        msg.value = estimator.voltage_to_soc(data.voltage) * 100.0
    else:
        # Fallback: simple linear estimate for 3S LiPo (9.0V - 12.6V)
        soc = (data.voltage - 9.0) / (12.6 - 9.0)
        msg.value = max(0.0, min(100.0, soc * 100.0))
    
    return msg


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert Euler angles to quaternion.
    
    Args:
        roll: Roll angle in radians
        pitch: Pitch angle in radians
        yaw: Yaw angle in radians
        
    Returns:
        Tuple (x, y, z, w) quaternion
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    
    return (x, y, z, w)


def wrap_angle(angle: float) -> float:
    """
    Wrap angle to [-pi, pi].
    
    Args:
        angle: Angle in radians
        
    Returns:
        Wrapped angle in radians
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class MessageCallback:
    """
    Hardware message processor for ROS2 integration.
    
    Handles incoming data from the rover and publishes to ROS2 topics.
    
    Args:
        parent: ROS2 parent node with publishers
    """
    
    def __init__(self, parent):
        self.parent = parent
    
    def on_imu(self, data: IMUData) -> None:
        """Handle IMU data - publish to IMU and Temperature topics."""
        try:
            # Publish IMU
            imu_msg = imu_to_ros_msg(self.parent, data)
            self.parent.imu_pub.publish(imu_msg)
            
            # Publish temperature
            temp_msg = temp_to_ros_msg(self.parent, data.temp)
            self.parent.temp_pub.publish(temp_msg)
            
        except Exception as e:
            self.parent.get_logger().debug(f"IMU publish error: {e}")
    
    def on_power(self, data: PowerData) -> None:
        """Handle power data - publish to Battery topic."""
        try:
            battery_msg = power_to_battery_msg(
                self.parent, 
                data, 
                getattr(self.parent, 'battery_estimator', None)
            )
            self.parent.battery_pub.publish(battery_msg)
            
        except Exception as e:
            self.parent.get_logger().debug(f"Power publish error: {e}")
    
    def on_system(self, msg) -> None:
        """Handle system messages."""
        self.parent.get_logger().info(f"[{msg.module}] {msg.message}")
    
    def on_error(self, msg: str) -> None:
        """Handle error messages."""
        self.parent.get_logger().warn(f"Rover error: {msg}")