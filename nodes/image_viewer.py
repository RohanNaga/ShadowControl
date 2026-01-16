#!/usr/bin/env python3
"""
Image Viewer Node

Subscribes to the annotated pose image and displays it using OpenCV.

Usage:
  ros2 run shadow_control image_viewer
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageViewerNode(Node):
    """Displays the annotated pose estimation video stream."""
    
    def __init__(self):
        super().__init__('image_viewer_node')
        
        self.declare_parameter('image_topic', '/shadow_control/pose_image')
        image_topic = self.get_parameter('image_topic').value
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        self.get_logger().info(f'Image viewer started, listening to: {image_topic}')
        self.get_logger().info('Press ESC in the image window to quit')
    
    def image_callback(self, msg):
        """Display incoming image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('Shadow Control - Pose Estimation', cv_image)
            
            # Check for ESC key
            if cv2.waitKey(1) & 0xFF == 27:
                self.get_logger().info('ESC pressed, shutting down...')
                cv2.destroyAllWindows()
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Failed to display image: {e}')
    
    def destroy_node(self):
        """Clean up."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImageViewerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
