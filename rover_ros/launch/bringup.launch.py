"""
Launch file for PyRover ROS2 driver.

Usage:
    ros2 launch ros_rover bringup.launch.py
    
    # With custom params:
    ros2 launch rover_ros bringup.launch.py port:=/dev/ttyUSB0
    
    # With custom config file:
    ros2 launch rover_ros bringup.launch.py config:=/path/to/my_params.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('rover_ros')
    default_config = os.path.join(pkg_dir, 'config', 'rover_params.yaml')
    
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to rover parameters YAML file'
    )
    
    
    # Create rover node
    rover_node = Node(
        package='rover_ros',
        executable='rover_controller',
        name='rover_controller',
        output='screen',
        parameters=[LaunchConfiguration('config')],
        remappings=[
            # Add remappings here if needed
            # ('cmd_vel', '/my_robot/cmd_vel'),
        ]
    )

    # Create pose estimator node
    pose_estimator = Node(
        package='rover_ros',
        executable='pose_estimator',
        name='pose_estimator',
        output='screen',
        parameters=[
            LaunchConfiguration('config')],
        remappings=[
            # Add remappings here if needed
            # ('cmd_vel', '/my_robot/cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        config_arg,
        rover_node,
        pose_estimator
    ])
