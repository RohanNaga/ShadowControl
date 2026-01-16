#!/usr/bin/env python3
"""
Simple Pose Subscriber Node

Subscribes to pose data and prints joint angles to console.
Good for testing and debugging.

Usage:
  ros2 run shadow_control pose_subscriber
"""

import rclpy
from rclpy.node import Node
from shadow_control.msg import PoseData


class PoseSubscriberNode(Node):
    """Simple subscriber that prints pose data."""
    
    def __init__(self):
        super().__init__('pose_subscriber_node')
        
        self.declare_parameter('pose_topic', '/shadow_control/pose')
        pose_topic = self.get_parameter('pose_topic').value
        
        self.subscription = self.create_subscription(
            PoseData,
            pose_topic,
            self.pose_callback,
            10
        )
        
        self.get_logger().info(f'Pose subscriber started, listening to: {pose_topic}')
        self.get_logger().info('Press Ctrl+C to stop')
    
    def pose_callback(self, msg):
        """Handle incoming pose data."""
        if msg.pose_detected:
            self.get_logger().info(
                f'Frame {msg.frame_id:5d} | '
                f'L Abd: {msg.left.shoulder_abduction:6.1f}° | '
                f'L Ext: {msg.left.shoulder_extension:6.1f}° | '
                f'L Flex: {msg.left.elbow_flexion:6.1f}° | '
                f'R Abd: {msg.right.shoulder_abduction:6.1f}° | '
                f'R Ext: {msg.right.shoulder_extension:6.1f}° | '
                f'R Flex: {msg.right.elbow_flexion:6.1f}°'
            )
        else:
            self.get_logger().warning(f'Frame {msg.frame_id}: No pose detected')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PoseSubscriberNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
