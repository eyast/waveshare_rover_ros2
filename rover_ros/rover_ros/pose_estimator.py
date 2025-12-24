#!/usr/bin/env python3
"""
ROS2 Localization Node
======================

This node performs localization of the robot using different techniques,
depending on the configuration provided in `config\rover_params.yaml`.

The localiation results can be visualized in RViz as markers.

Valid configurations:
=====================
- Dead Reckoning: Uses Integration and Direct Euler to estimate the position of
  the bot. Disregard IMU or any sensor, rely only on commands sent to 'cmd_vel'
- IMU: Uses the onboard 9-DOF IMU to estimate the position of the bot. Disregards
  any command sent to 'cmd_vel'
- Bayes Filter: Fuses Dead_reckoning and IMU using a Bayes Filter.
- Kalman Filter: Fuses Dead_reckoning and IMU using a Kalman Filter.

Subscriptions
-------------
cmd_vel (geometry_msgs/Twist)
    Retrieves velocity commands sent to the robot

sensor/IMU (rover_msgs/IMU)
    IMU data retrieved from the onboard 9-DOF chip

Publishers
-----------
state (rover_msgs/State)
    Estimate of the current Robot state

tf2 tree
    TF broadcaster
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
    """ROS2 Pose Estimator for the Waveshare rover"""

    def __init__(self):
        super().__init__("pose_estimator")
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 20.0) 
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('localization_mode','dead_reckoning')
    
        # ===== Get Parameters =======
        self.publish_tf: bool = self.get_parameter('publish_tf').value # type: ignore
        self.publish_rate: float = self.get_parameter('publish_rate').value # type: ignore
        self.frame_id: str = self.get_parameter('frame_id').value # type: ignore
        self.odom_frame_id: str = self.get_parameter('odom_frame_id').value # type: ignore
        self.localization_mode: str = self.get_parameter('localization_mode').value # type: ignore
        self.get_logger().info(f"PoseEstimater started in {self.localization_mode} mode")
        # ===== Internal State =======
        self.state = State()
        self.latest_twist = Twist()
        self.latest_imu = IMUv2()
        self.last_time = self.get_clock().now()
        
        # ====== TF Broadcaster ======
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().info(f"TF: {self.odom_frame_id} -> {self.frame_id}")
        
        # ===== Publishers ===========
        self.state_pub = self.create_publisher(State, 'state', 10)

        # ==== Subscriptions =========
        self.cmd_vel_sub = self.create_subscription(Twist,
                                                    'cmd_vel',
                                                    self._cmd_vel_cb,
                                                    10)

        self.imu_sub = self.create_subscription(IMUv2,
                                                'sensor/IMU',
                                                self._imu_cb,
                                                10)

        # ======= Timer ==============
        self.polling_timer = self.create_timer(1 / self.publish_rate,
                                                    self._update_odometry)

    def _update_odometry(self):
        """Function called by the timer.
        Updates the Pose of the robot based on the localization selected.
        The updated pose is published to /state, and as a tf tree"""
        v, om, x, y, theta, now, dt = self._get_actuals()
        state_new = State()
        state_new.header.frame_id = self.frame_id
        state_new.header.stamp = self.get_clock().now().to_msg()
        if self.localization_mode == "dead_reckoning":
            x_new, y_new, th_new = self._predict(x,
                                                 y,
                                                 theta,
                                                 v,
                                                 om,
                                                 dt)
            state_new.x = x_new
            state_new.y = y_new
            state_new.theta = th_new
        if self.localization_mode == "imu":
            x_new, y_new, _ = self._predict(x,
                                            y,
                                            theta,
                                            v,
                                            om,
                                            dt)
            state_new.x = x_new
            state_new.y = y_new
            state_new.theta = np.deg2rad (self.latest_imu.yaw)
        self.state = state_new
        self.state_pub.publish(state_new)
        self._publish_tf(now)
        self.last_time = now

    def _predict(self, x, y, theta, v, om, dt):
        """Performs a Discretized Euler prediction"""
        x_new= x + v * math.cos(theta) * dt
        y_new = y + v * math.sin(theta) * dt
        th_new = wrap_angle(theta + om * dt)
        return x_new, y_new, th_new

    def _get_actuals(self):
        """Support function to make _update_odometry()
        look slimmer.
        """
        v = self.latest_twist.linear.x
        om = self.latest_twist.angular.z
        x = self.state.x
        y = self.state.y
        theta = self.state.theta
        now = self.get_clock().now()
        #dt = 1 / ((now.nanoseconds - self.last_time.nanoseconds) / 1e9)
        dt = 1 / self.publish_rate
        return v,om,x,y,theta,now,dt

    def _cmd_vel_cb(self, msg: Twist):
        """Updates variables with latest commands issued to cmd_vel."""
        self.latest_twist = msg

    def _imu_cb(self, msg: IMUv2):
        """Updates internal variables to support other functions."""
        self.latest_imu = msg

    def _publish_tf(self, stamp):
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