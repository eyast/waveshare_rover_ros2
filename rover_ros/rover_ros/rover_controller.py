#!/usr/bin/env python3
"""
ROS2 PyRover Driver Node (New Line-Based Protocol)
===================================================

This node provides a ROS2 interface for the Waveshare WAVE ROVER robot
using the new line-based serial protocol.

Subscriptions
-------------
cmd_vel (geometry_msgs/Twist)
    Velocity commands for differential drive.

Publishers
-----------
sensor/IMU (rover_msgs/IMUv2)
    IMU data with orientation and raw sensor values

sensor/Temperature (rover_msgs/Temperature)
    CPU temperature

sensor/Battery (rover_msgs/Battery)
    Battery level percentage

Parameters
----------
port: Serial port (default: '/dev/serial0')
baudrate: Serial baudrate (default: 115200)
max_speed: Maximum PWM output (default: 255)
linear_gain: Linear velocity to PWM gain (default: 200)
angular_gain: Angular velocity to PWM gain (default: 100)
publish_imu: Enable IMU publishing (default: True)
publish_battery: Enable battery publishing (default: True)
frame_id: TF frame ID (default: 'base_link')
battery_cells: Number of battery cells (default: 3)
battery_capacity_mah: Battery capacity in mAh (default: 2600)
"""

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist

from pyrover import PyRover, RoverCallbacks, BatteryEstimator, IMUData, PowerData
from rover_msgs.msg import IMUv2, Temperature, Battery
from rover_ros.tools import MessageCallback


class RoverController(Node):
    """ROS2 node for PyRover with line-based protocol."""
    
    def __init__(self):
        super().__init__("rover_controller")
        
        # ===== Declare Parameters =====
        self.declare_parameter('port', '/dev/serial0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_speed', 255)
        self.declare_parameter('linear_gain', 200.0)
        self.declare_parameter('angular_gain', 100.0)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('publish_battery', True)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('battery_cells', 3)
        self.declare_parameter('battery_capacity_mah', 2600)
        
        # ===== Get Parameters =====
        self.port: str = self.get_parameter('port').value  # type: ignore
        self.baudrate: int = self.get_parameter('baudrate').value  # type: ignore
        self.max_speed: int = self.get_parameter('max_speed').value  # type: ignore
        self.linear_gain: float = self.get_parameter('linear_gain').value  # type: ignore
        self.angular_gain: float = self.get_parameter('angular_gain').value  # type: ignore
        self.publish_imu: bool = self.get_parameter('publish_imu').value  # type: ignore
        self.publish_battery: bool = self.get_parameter('publish_battery').value  # type: ignore
        self.frame_id: str = self.get_parameter('frame_id').value  # type: ignore
        battery_cells: int = self.get_parameter('battery_cells').value  # type: ignore
        battery_capacity: int = self.get_parameter('battery_capacity_mah').value  # type: ignore
        
        # ===== State =====
        self.last_cmd_time = self.get_clock().now()
        self.v: float = 0.0
        self.om: float = 0.0
        
        # ===== Battery Estimator =====
        self.battery_estimator = BatteryEstimator(
            num_cells=battery_cells,
            capacity_mah=battery_capacity
        )
        
        # ===== Publishers =====
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.imu_pub = self.create_publisher(IMUv2, '/sensor/IMU', 10)
        self.temp_pub = self.create_publisher(Temperature, '/sensor/Temperature', 10)
        self.battery_pub = self.create_publisher(Battery, '/sensor/Battery', 10)
        # ===== Subscribers =====
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 10
        )
        
        # ===== Message Callback Handler =====
        self._msg_handler = MessageCallback(self)
        
        # ===== Connect to Robot =====
        self.rover: Optional[PyRover] = None
        self._connect_robot()
        
        # ===== Heartbeat Timer =====
        self.create_timer(1.0, self._heartbeat_check)
        
        self.get_logger().info(f"RoverController started on {self.port}")
    
    def _connect_robot(self) -> None:
        """Connect to the rover."""
        try:
            # Set up callbacks
            callbacks = RoverCallbacks(
                on_imu=self._msg_handler.on_imu if self.publish_imu else None,
                on_power=self._msg_handler.on_power if self.publish_battery else None,
                on_system=self._msg_handler.on_system,
            )
            self.rover = PyRover(
                port=self.port,
                baudrate=self.baudrate,
                callbacks=callbacks,
            )
            self.rover.connect()
            
            # Enable streaming in IMU format
            self.rover.stream_on()
            self.rover.set_format_imu()
            
            self.get_logger().info(f"Connected to PyRover on {self.port}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            self.rover = None
    
    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Handle velocity commands."""
        if self.rover is None or not self.rover.is_connected:
            self.get_logger().warn('Cannot send cmd_vel: not connected')
            return
        
        self.last_cmd_time = self.get_clock().now()
        
        linear = msg.linear.x
        angular = msg.angular.z
        self.v = linear
        self.om = angular
        
        # Convert to wheel PWM using differential drive kinematics
        # left = linear - angular (positive angular = turn left)
        # right = linear + angular
        left_pwm = (linear * self.linear_gain) - (angular * self.angular_gain)
        right_pwm = (linear * self.linear_gain) + (angular * self.angular_gain)
        
        # Clamp to max speed while preserving ratio
        max_pwm = max(abs(left_pwm), abs(right_pwm))
        if max_pwm > self.max_speed:
            scale = self.max_speed / max_pwm
            left_pwm *= scale
            right_pwm *= scale
        
        self.rover.move(int(left_pwm), int(right_pwm))
    
    def _heartbeat_check(self) -> None:
        """Periodic check - stop if no commands received."""
        if self.rover is None or not self.rover.is_connected:
            return
        
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
        
        # Stop if no command for heartbeat timeout and we were moving
        if elapsed >= PyRover.HEARTBEAT_TIMEOUT and (self.v != 0 or self.om != 0):
            self.get_logger().info("Heartbeat timeout - stopping motors")
            self.rover.stop()
            self.v = 0.0
            self.om = 0.0
            
            # Publish stop command
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
    
    def destroy_node(self) -> None:
        """Clean shutdown."""
        if self.rover is not None and self.rover.is_connected:
            self.rover.stop()
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