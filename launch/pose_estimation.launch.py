#!/usr/bin/env python3
"""
Launch file for Shadow Control pose estimation system.

Starts the pose publisher and optionally the image viewer.

Usage:
  ros2 launch shadow_control pose_estimation.launch.py
  ros2 launch shadow_control pose_estimation.launch.py camera_id:=0
  ros2 launch shadow_control pose_estimation.launch.py show_image:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='1',
        description='Camera device ID'
    )
    
    show_image_arg = DeclareLaunchArgument(
        'show_image',
        default_value='true',
        description='Whether to show annotated image window'
    )
    
    publish_image_arg = DeclareLaunchArgument(
        'publish_image',
        default_value='true',
        description='Whether to publish annotated images'
    )
    
    # Pose publisher node
    pose_publisher_node = Node(
        package='shadow_control',
        executable='pose_publisher',
        name='pose_publisher',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'publish_image': LaunchConfiguration('publish_image'),
        }]
    )
    
    # Image viewer node (conditional based on show_image)
    image_viewer_node = Node(
        package='shadow_control',
        executable='image_viewer',
        name='image_viewer',
        output='screen',
        condition=IfCondition(
            LaunchConfiguration('show_image')
        )
    )
    
    return LaunchDescription([
        camera_id_arg,
        show_image_arg,
        publish_image_arg,
        pose_publisher_node,
        image_viewer_node,
    ])
