#!/usr/bin/env python3
"""
Complete system launch file.

Launches pose estimation + robot controller.

Usage:
  ros2 launch shadow_control system.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Pose publisher node
    pose_publisher_node = Node(
        package='shadow_control',
        executable='pose_publisher',
        name='pose_publisher',
        output='screen',
        parameters=[{
            'camera_id': 1,
            'publish_image': True,
        }]
    )
    
    # Robot controller node
    robot_controller_node = Node(
        package='shadow_control',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[{
            'control_rate_hz': 50.0,
        }]
    )
    
    # Optional: Simple subscriber for debugging
    pose_subscriber_node = Node(
        package='shadow_control',
        executable='pose_subscriber',
        name='pose_subscriber',
        output='screen',
    )
    
    return LaunchDescription([
        pose_publisher_node,
        robot_controller_node,
        # Uncomment to add debug subscriber:
        # pose_subscriber_node,
    ])
