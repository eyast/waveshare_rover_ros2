#!/usr/bin/env python3
"""
ROS2 Localization Node
======================

This node performs localization of the robot using different techniques.

Localization Modes:
- dead_reckoning: Uses cmd_vel integration (no sensors)
- imu: Uses IMU yaw for heading
- fused: Combines dead reckoning with IMU correction

Subscriptions
-------------
cmd_vel (geometry_msgs/Twist)
    Velocity commands sent to the robot

sensor/IMU (rover_msgs/IMUv2)
    IMU data with orientation

Publishers
-----------
state (rover_msgs/State)
    Estimated robot state (x, y, theta)

tf2 tree
    odom -> base_link transform
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

from rover_msgs.msg import IMUv2, State
from pyrover.tools import quaternion_from_euler, wrap_angle


class PoseEstimate(Node):
    """ROS2 Pose Estimator for the Waveshare rover."""
    
    def __init__(self):
        super().__init__("pose_estimator")
        
        # ===== Parameters =====
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('localization_mode', 'dead_reckoning')
        
        self.publish_tf: bool = self.get_parameter('publish_tf').value  # type: ignore
        self.publish_rate: float = self.get_parameter('publish_rate').value  # type: ignore
        self.frame_id: str = self.get_parameter('frame_id').value  # type: ignore
        self.odom_frame_id: str = self.get_parameter('odom_frame_id').value  # type: ignore
        self.localization_mode: str = self.get_parameter('localization_mode').value  # type: ignore
        
        self.get_logger().info(f"PoseEstimator started in {self.localization_mode} mode")
        
        # ===== State =====
        self.state = State()
        self.latest_twist = Twist()
        self.latest_imu = IMUv2()
        self.last_time = self.get_clock().now()
        self.imu_yaw_offset: float = 0.0  # Offset to align IMU yaw with odometry
        self.first_imu = True
        
        # ===== TF Broadcaster =====
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().info(f"TF: {self.odom_frame_id} -> {self.frame_id}")
        
        # ===== Publishers =====
        self.state_pub = self.create_publisher(State, 'state', 10)
        
        # ===== Subscriptions =====
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_cb, 10
        )
        self.imu_sub = self.create_subscription(
            IMUv2, '/sensor/IMU', self._imu_cb, 10
        )
        
        # ===== Timer =====
        self.create_timer(1.0 / self.publish_rate, self._update_odometry)
    
    def _update_odometry(self) -> None:
        """Update pose estimate and publish."""
        now = self.get_clock().now()
        dt = 1.0 / self.publish_rate  # Fixed dt for stability
        
        v = self.latest_twist.linear.x
        om = self.latest_twist.angular.z
        x = self.state.x
        y = self.state.y
        theta = self.state.theta
        
        # Create new state message
        state_new = State()
        state_new.header.frame_id = self.frame_id
        state_new.header.stamp = now.to_msg()
        
        if self.localization_mode == "dead_reckoning":
            # Pure dead reckoning
            x_new, y_new, th_new = self._predict(x, y, theta, v, om, dt)
            state_new.x = x_new
            state_new.y = y_new
            state_new.theta = th_new
            
        elif self.localization_mode == "imu":
            # Use IMU for heading, dead reckoning for position
            x_new, y_new, _ = self._predict(x, y, theta, v, om, dt)
            state_new.x = x_new
            state_new.y = y_new
            # Convert IMU yaw (degrees) to radians and apply offset
            imu_yaw_rad = np.deg2rad(self.latest_imu.y) - self.imu_yaw_offset
            state_new.theta = wrap_angle(imu_yaw_rad)
            
        elif self.localization_mode == "fused":
            # Simple fusion: predict with dead reckoning, correct with IMU
            x_new, y_new, th_pred = self._predict(x, y, theta, v, om, dt)
            state_new.x = x_new
            state_new.y = y_new
            
            # Blend IMU and predicted heading (70% IMU, 30% prediction)
            imu_yaw_rad = np.deg2rad(self.latest_imu.y) - self.imu_yaw_offset
            alpha = 0.7
            th_fused = wrap_angle(alpha * imu_yaw_rad + (1 - alpha) * th_pred)
            state_new.theta = th_fused
        
        else:
            # Default to dead reckoning
            x_new, y_new, th_new = self._predict(x, y, theta, v, om, dt)
            state_new.x = x_new
            state_new.y = y_new
            state_new.theta = th_new
        
        self.state = state_new
        self.state_pub.publish(state_new)
        
        if self.publish_tf:
            self._publish_tf(now)
        
        self.last_time = now
    
    def _predict(self, x: float, y: float, theta: float, 
                 v: float, om: float, dt: float) -> tuple:
        """
        Discretized Euler prediction for differential drive.
        
        Returns:
            Tuple of (x_new, y_new, theta_new)
        """
        x_new = x + v * math.cos(theta) * dt
        y_new = y + v * math.sin(theta) * dt
        th_new = wrap_angle(theta + om * dt)
        return x_new, y_new, th_new
    
    def _cmd_vel_cb(self, msg: Twist) -> None:
        """Update latest velocity command."""
        self.latest_twist = msg
    
    def _imu_cb(self, msg: IMUv2) -> None:
        """Update latest IMU data."""
        if self.first_imu:
            # Initialize yaw offset so IMU matches current odometry heading
            self.imu_yaw_offset = np.deg2rad(msg.y) - self.state.theta
            self.first_imu = False
            self.get_logger().info(f"IMU yaw offset set to {np.rad2deg(self.imu_yaw_offset):.1f}°")
        
        self.latest_imu = msg
    
    def _publish_tf(self, stamp) -> None:
        """Publish odom -> base_link transform."""
        q = quaternion_from_euler(0, 0, self.state.theta)
        
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.frame_id
        t.transform.translation.x = self.state.x
        t.transform.translation.y = self.state.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down PoseEstimator...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()