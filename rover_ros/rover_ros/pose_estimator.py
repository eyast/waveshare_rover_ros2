#!/usr/bin/env python3
"""
ROS2 Pose Estimator Node (Line-Based Protocol)
===============================================

Performs robot localization using dead reckoning, IMU fusion, or combined methods.

Localization Modes
------------------
- dead_reckoning: Pure cmd_vel integration (no sensors, drifts over time)
- imu: Uses IMU yaw for heading, cmd_vel for position
- fused: Weighted blend of dead reckoning and IMU (configurable alpha)

Subscriptions
-------------
cmd_vel (geometry_msgs/Twist)
    Velocity commands - used for position integration

/sensor/IMU (rover_msgs/IMUv2)
    IMU data with yaw/pitch/roll orientation

Publishers
----------
state (rover_msgs/State)
    Estimated robot state (x, y, theta)

pose_marker (visualization_msgs/Marker)
    Arrow marker for RViz visualization

TF2
---
Broadcasts: odom -> base_link transform

Parameters
----------
publish_tf : bool (default: True)
    Whether to publish TF transform

publish_rate : float (default: 20.0)
    Rate at which to update and publish odometry

frame_id : str (default: 'base_link')
    Robot body frame

odom_frame_id : str (default: 'odom')
    Odometry frame

localization_mode : str (default: 'fused')
    One of: 'dead_reckoning', 'imu', 'fused'

fusion_alpha : float (default: 0.7)
    IMU weight for fused mode (0.0 = pure DR, 1.0 = pure IMU)

arrow_scale : float (default: 0.3)
    Size of the RViz arrow marker
"""

import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster

from rover_msgs.msg import IMUv2, State
from pyrover.tools import quaternion_from_euler, wrap_angle


class PoseEstimator(Node):
    """ROS2 Pose Estimator with IMU fusion and RViz visualization."""
    
    def __init__(self):
        super().__init__("pose_estimator")
        
        # =====================================================================
        # Parameters
        # =====================================================================
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('localization_mode', 'fused')
        self.declare_parameter('fusion_alpha', 0.7)
        self.declare_parameter('arrow_scale', 0.3)
        self.declare_parameter('publish_marker', True)
        
        self.publish_tf: bool = self.get_parameter('publish_tf').value
        self.publish_rate: float = self.get_parameter('publish_rate').value
        self.frame_id: str = self.get_parameter('frame_id').value
        self.odom_frame_id: str = self.get_parameter('odom_frame_id').value
        self.localization_mode: str = self.get_parameter('localization_mode').value
        self.fusion_alpha: float = self.get_parameter('fusion_alpha').value
        self.arrow_scale: float = self.get_parameter('arrow_scale').value
        self.publish_marker: bool = self.get_parameter('publish_marker').value
        
        self.get_logger().info(f"PoseEstimator started")
        self.get_logger().info(f"  Mode: {self.localization_mode}")
        self.get_logger().info(f"  Fusion alpha: {self.fusion_alpha}")
        self.get_logger().info(f"  Publish TF: {self.publish_tf}")
        self.get_logger().info(f"  Publish marker: {self.publish_marker}")
        
        # =====================================================================
        # State
        # =====================================================================
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0  # radians
        
        self.latest_linear: float = 0.0
        self.latest_angular: float = 0.0
        self.latest_imu_yaw: float = 0.0  # degrees from IMU
        
        self.imu_yaw_offset: float = 0.0  # Offset to align IMU with odometry
        self.first_imu: bool = True
        self.imu_received: bool = False
        
        self.last_time = self.get_clock().now()
        
        # =====================================================================
        # TF Broadcaster
        # =====================================================================
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().info(f"  TF: {self.odom_frame_id} -> {self.frame_id}")
        
        # =====================================================================
        # Publishers
        # =====================================================================
        self.state_pub = self.create_publisher(State, 'state', 10)
        
        if self.publish_marker:
            self.marker_pub = self.create_publisher(Marker, 'pose_marker', 10)
        
        # =====================================================================
        # Subscriptions
        # =====================================================================
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_cb, 10
        )
        self.imu_sub = self.create_subscription(
            IMUv2, '/sensor/IMU', self._imu_cb, 10
        )
        
        # =====================================================================
        # Timer
        # =====================================================================
        self.create_timer(1.0 / self.publish_rate, self._update_loop)
    
    # =========================================================================
    # Main Update Loop
    # =========================================================================
    
    def _update_loop(self) -> None:
        """Update pose estimate and publish all outputs."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        
        # Clamp dt to reasonable range
        if dt <= 0 or dt > 0.5:
            dt = 1.0 / self.publish_rate
        
        # Get current velocities
        v = self.latest_linear
        omega = self.latest_angular
        
        # Update pose based on localization mode
        if self.localization_mode == "dead_reckoning":
            self._update_dead_reckoning(v, omega, dt)
            
        elif self.localization_mode == "imu":
            self._update_imu_mode(v, omega, dt)
            
        elif self.localization_mode == "fused":
            self._update_fused_mode(v, omega, dt)
            
        else:
            # Default to dead reckoning
            self._update_dead_reckoning(v, omega, dt)
        
        # Publish state message
        self._publish_state(now)
        
        # Publish TF
        if self.publish_tf:
            self._publish_tf(now)
        
        # Publish RViz marker
        if self.publish_marker:
            self._publish_arrow_marker(now)
        
        self.last_time = now
    
    # =========================================================================
    # Localization Methods
    # =========================================================================
    
    def _update_dead_reckoning(self, v: float, omega: float, dt: float) -> None:
        """Pure dead reckoning - integrate velocities."""
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = wrap_angle(self.theta + omega * dt)
    
    def _update_imu_mode(self, v: float, omega: float, dt: float) -> None:
        """Use IMU for heading, dead reckoning for position."""
        # Position from dead reckoning
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        
        # Heading from IMU (if available)
        if self.imu_received:
            imu_yaw_rad = math.radians(self.latest_imu_yaw) - self.imu_yaw_offset
            self.theta = wrap_angle(imu_yaw_rad)
        else:
            # Fall back to dead reckoning for heading too
            self.theta = wrap_angle(self.theta + omega * dt)
    
    def _update_fused_mode(self, v: float, omega: float, dt: float) -> None:
        """Weighted fusion of dead reckoning and IMU."""
        # Position from dead reckoning
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        
        # Predict heading from dead reckoning
        theta_predicted = wrap_angle(self.theta + omega * dt)
        
        if self.imu_received:
            # Get IMU heading
            imu_yaw_rad = math.radians(self.latest_imu_yaw) - self.imu_yaw_offset
            imu_yaw_wrapped = wrap_angle(imu_yaw_rad)
            
            # Handle angle wraparound for fusion
            # Convert to complex numbers for proper averaging
            pred_x = math.cos(theta_predicted)
            pred_y = math.sin(theta_predicted)
            imu_x = math.cos(imu_yaw_wrapped)
            imu_y = math.sin(imu_yaw_wrapped)
            
            # Weighted average
            alpha = self.fusion_alpha
            fused_x = alpha * imu_x + (1 - alpha) * pred_x
            fused_y = alpha * imu_y + (1 - alpha) * pred_y
            
            self.theta = math.atan2(fused_y, fused_x)
        else:
            self.theta = theta_predicted
    
    # =========================================================================
    # Callbacks
    # =========================================================================
    
    def _cmd_vel_cb(self, msg: Twist) -> None:
        """Store latest velocity command."""
        self.latest_linear = msg.linear.x
        self.latest_angular = msg.angular.z
    
    def _imu_cb(self, msg: IMUv2) -> None:
        """Store latest IMU data and initialize offset on first message."""
        if self.first_imu:
            # Set offset so IMU aligns with current odometry heading
            # IMU yaw is in degrees, convert to radians
            self.imu_yaw_offset = math.radians(msg.y) - self.theta
            self.first_imu = False
            self.get_logger().info(
                f"IMU initialized: raw_yaw={msg.y:.1f}°, offset={math.degrees(self.imu_yaw_offset):.1f}°"
            )
        
        self.latest_imu_yaw = msg.y
        self.imu_received = True
    
    # =========================================================================
    # Publishers
    # =========================================================================
    
    def _publish_state(self, stamp) -> None:
        """Publish State message."""
        msg = State()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        self.state_pub.publish(msg)
    
    def _publish_tf(self, stamp) -> None:
        """Publish odom -> base_link transform."""
        q = quaternion_from_euler(0.0, 0.0, self.theta)
        
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def _publish_arrow_marker(self, stamp) -> None:
        """Publish arrow marker for RViz visualization."""
        marker = Marker()
        marker.header.stamp = stamp.to_msg()
        marker.header.frame_id = self.odom_frame_id
        marker.ns = "pose_estimator"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.05  # Slightly above ground
        
        # Orientation (from heading)
        q = quaternion_from_euler(0.0, 0.0, self.theta)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        
        # Scale (length, width, height of arrow)
        marker.scale.x = self.arrow_scale  # Arrow length
        marker.scale.y = self.arrow_scale * 0.2  # Arrow width
        marker.scale.z = self.arrow_scale * 0.2  # Arrow height
        
        # Color (green arrow)
        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 1.0
        
        # Lifetime (0 = forever, but we publish continuously)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = int(0.2 * 1e9)  # 200ms
        
        self.marker_pub.publish(marker)
    
    # =========================================================================
    # Service Methods (for future use)
    # =========================================================================
    
    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """Reset pose to specified values."""
        self.x = x
        self.y = y
        self.theta = theta
        self.first_imu = True  # Re-initialize IMU offset
        self.get_logger().info(f"Pose reset to ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}°)")


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down PoseEstimator...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()