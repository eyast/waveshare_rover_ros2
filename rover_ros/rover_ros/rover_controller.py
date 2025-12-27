#!/usr/bin/env python3
"""
ROS2 PyRover Driver Node
========================

This node provides a ROS2 interface for the Waveshare WAVE ROVER robot.
It uses the pyrover library for serial communication and the Mahony AHRS
filter for accurate heading estimation with magnetometer calibration.

Subscriptions
-------------
cmd_vel (geometry_msgs/Twist)
    Velocity commands for differential drive.

Publishers
-----------
cmd_vel (geometry_msgs/Twist)
    Velocity commands for differential drive - publishes emergency stop

sensor/IMU (rover_msgs/IMU)
    IMU data retrieved from the onboard 9-DOF chip

sensor/Temperature (rover_msgs/Temperature)
    Board temperature

sensor/Battery (rover_msgs/Battery)
    Battery level, in percentage


Parameters
----------
See config/rover_params.yaml for full parameter documentation.
"""

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist

from pyrover import PyRover, BatteryEstimator, IMUData_v2
from rover_msgs.msg import (
    IMUv2,
    Temperature,
    Battery,
    Wheel
)
from rover_ros import MessageCallback


class RoverController(Node):
    """ROS2 node for PyRover"""
    
    def __init__(self):
        super().__init__("rover_controller")
        
        # ===== Declare Parameters =====
        # Serial connection
        self.declare_parameter('port', '/dev/serial0')
        self.declare_parameter('baudrate', 115200)
        
        # Robot geometry
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('linear_gain', 0.8)
        self.declare_parameter('angular_gain', 0.3)
        
        # Publishing
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('publish_battery', True)
        self.declare_parameter('publish_rate', 20.0) 
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        
        # Battery
        self.declare_parameter('battery_cells', 3)
        self.declare_parameter('battery_capacity_mah', 2600)
        
        # ===== Get Parameters =====
        self.port:int = self.get_parameter('port').value # type: ignore
        self.baudrate:int = self.get_parameter('baudrate').value # type: ignore
        self.max_speed: float = self.get_parameter('max_speed').value # type: ignore
        self.linear_gain: float = self.get_parameter('linear_gain').value # type: ignore
        self.angular_gain: float = self.get_parameter('angular_gain').value # type: ignore
        self.publish_imu: bool = self.get_parameter('publish_imu').value # type: ignore
        self.publish_battery: bool = self.get_parameter('publish_battery').value # type: ignore
        self.publish_rate: float = self.get_parameter('publish_rate').value # type: ignore
        self.frame_id: str = self.get_parameter('frame_id').value # type: ignore
        self.odom_frame_id: str = self.get_parameter('odom_frame_id').value # type: ignore
        battery_cells: int = self.get_parameter('battery_cells').value # type: ignore
        battery_capacity: int = self.get_parameter('battery_capacity_mah').value # type: ignore
        
        # Thread locks and Timekeepers
        self.last_time_cmd = self.get_clock().now()
        self.imu_lock = threading.Lock()
        self.battery_lock = threading.Lock()
        
        # ===== Connect to Robot =====
        self.rover: Optional[PyRover] = None
        self._connect_robot()
        
        # ==== Last commands sent =====
        self.v: float = 0.0
        self.om: float = 0.0

        # ===== Battery Estimator =====
        self.battery_estimator = BatteryEstimator(
            num_cells=battery_cells,
            capacity_mah=battery_capacity)
        
        # ===== Subscribers =====
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 10)
        
        # ===== Publisher =======
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)
        self.imu_pub = self.create_publisher(
        IMUv2, '/sensor/IMU', 10)
        self.temp_pub = self.create_publisher(
        Temperature, '/sensor/Temperature', 10)
        self.battery_pub = self.create_publisher(
        Battery, '/sensor/Battery', 10)
        self.wheel_pub = self.create_publisher(
        Wheel, '/sensor/Wheel', 10)

        # ===== HW callback function ===
        self._cb = MessageCallback(self)

    def _connect_robot(self):
        """Connect to the PyRover."""
        try:
            self.rover = PyRover(
                port=self.port, # type: ignore
                baudrate=self.baudrate,
                callback=self._cb.process # type: ignore
            )
            self.rover.connect()
            self.rover.imu_stream(True)
            self.rover.imu_stream_json(True)
            self.get_logger().info(f"Connected to PyRover on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            self.rover = None

    def _emergency_check(self):
        # Stop the bot to align with the low level heartbeat (every 3 seconds)
        # Stop if Temperature is too high
        now = self.get_clock().now()
        if (now - self.last_time_cmd)>= Duration(seconds=3) and \
        self.v != 0 and self.om !=0:
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
            self.v = 0.0
            self.om = 0.0
            self.last_time_cmd = now
    
    def _cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands."""
        if self.rover is None or not self.rover.is_connected:
            self.get_logger().warn('Cannot send cmd_vel: not connected')
            return
        
        self.last_time_cmd = self.get_clock().now()
        
        linear = msg.linear.x
        angular = msg.angular.z
        self.v = linear
        self.om = angular
        
        # Convert to wheel commands using calibrated gains
        left_cmd: float = (linear * self.linear_gain) - (angular * self.angular_gain)
        right_cmd: float = (linear * self.linear_gain) + (angular * self.angular_gain)
        
        # Clamp to max speed
        max_cmd: float = max(abs(left_cmd), abs(right_cmd))
        if max_cmd > self.max_speed: # type: ignore
            scale = self.max_speed / max_cmd # type: ignore
            left_cmd *= scale
            right_cmd *= scale
        
        self.rover.move(left_cmd, right_cmd)
    
    def destroy_node(self):
        """Clean shutdown."""
        if self.rover is not None and self.rover.is_connected:
            self.rover.move(0.0, 0.0)  # Stop motors
            self.rover.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoverController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down RoverController...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
