"""
Launch file for PyRover ROS2 Driver
===================================

Launches the rover controller and pose estimator nodes.

Usage:
    # Basic launch
    ros2 launch rover_ros bringup.launch.py
    
    # With custom serial port
    ros2 launch rover_ros bringup.launch.py port:=/dev/ttyUSB0
    
    # With custom config file
    ros2 launch rover_ros bringup.launch.py config:=/path/to/my_params.yaml
    
    # Disable pose estimator
    ros2 launch rover_ros bringup.launch.py enable_pose_estimator:=false
    
    # Change localization mode
    ros2 launch rover_ros bringup.launch.py localization_mode:=imu

Nodes:
    - rover_controller: Motor control and IMU streaming
    - pose_estimator: Localization with IMU fusion and TF publishing
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # =========================================================================
    # Get package directory and default config
    # =========================================================================
    pkg_dir = get_package_share_directory('rover_ros')
    default_config = os.path.join(pkg_dir, 'config', 'rover_params.yaml')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    # Config file
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to rover parameters YAML file'
    )
    
    # Serial port (override config file)
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyAMA0',
        description='Serial port override (empty = use config file)'
    )
    
    # Enable/disable pose estimator
    enable_pose_arg = DeclareLaunchArgument(
        'enable_pose_estimator',
        default_value='true',
        description='Enable pose estimator node'
    )
    
    # Localization mode override
    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode',
        default_value='fused',
        description='Localization mode: dead_reckoning, imu, or fused'
    )
    
    # =========================================================================
    # Rover Controller Node
    # =========================================================================
    rover_controller_node = Node(
        package='rover_ros',
        executable='rover_controller',
        name='rover_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            # Override port if specified
            {'port': LaunchConfiguration('port')},
        ],
        remappings=[
            # Standard remappings
            # ('cmd_vel', '/cmd_vel'),
        ],
    )
    
    # =========================================================================
    # Pose Estimator Node
    # =========================================================================
    pose_estimator_node = Node(
        package='rover_ros',
        executable='pose_estimator',
        name='pose_estimator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_pose_estimator')),
        parameters=[
            LaunchConfiguration('config'),
            {'localization_mode': LaunchConfiguration('localization_mode')},
        ],
        remappings=[
            # Remap IMU topic if needed
            # ('/sensor/IMU', '/imu/data'),
        ],
    )
    
    # =========================================================================
    # Static Transform: base_link -> imu_link (if IMU is offset from base)
    # =========================================================================
    # Uncomment if your IMU is not at the robot center
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_to_imu_tf',
    #     arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link'],
    # )
    
    # =========================================================================
    # Return Launch Description
    # =========================================================================
    return LaunchDescription([
        # Arguments
        config_arg,
        port_arg,
        enable_pose_arg,
        localization_mode_arg,
        
        # Nodes
        rover_controller_node,
        pose_estimator_node,
        # static_tf_node,
    ])