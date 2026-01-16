#!/usr/bin/env python3
"""
Robot Control Subscriber Node (Template)

This is a template for creating a robot control node that subscribes
to pose data and controls the robot accordingly.

Modify the control_callback() method to implement your robot control logic.

Usage:
  ros2 run shadow_control robot_controller
"""

import rclpy
from rclpy.node import Node
from shadow_control.msg import PoseData


class RobotControllerNode(Node):
    """
    Subscribes to pose data and controls the robot.
    
    This is a template - implement your actual robot control logic here.
    """
    
    def __init__(self):
        super().__init__('robot_controller_node')
        
        self.declare_parameter('pose_topic', '/shadow_control/pose')
        self.declare_parameter('control_rate_hz', 50.0)
        
        pose_topic = self.get_parameter('pose_topic').value
        control_rate = self.get_parameter('control_rate_hz').value
        
        # Subscribe to pose data
        self.subscription = self.create_subscription(
            PoseData,
            pose_topic,
            self.pose_callback,
            10
        )
        
        # Store latest pose data
        self.latest_pose = None
        
        # Create control loop timer
        self.control_timer = self.create_timer(
            1.0 / control_rate,
            self.control_callback
        )
        
        self.get_logger().info(f'Robot controller started')
        self.get_logger().info(f'  Listening to: {pose_topic}')
        self.get_logger().info(f'  Control rate: {control_rate} Hz')
        self.get_logger().info('  ** This is a template - implement your control logic **')
    
    def pose_callback(self, msg):
        """Store latest pose data."""
        self.latest_pose = msg
    
    def control_callback(self):
        """
        Main control loop - called at fixed rate.
        
        Implement your robot control logic here:
        1. Read self.latest_pose
        2. Map joint angles to robot commands
        3. Send commands to robot
        """
        if self.latest_pose is None:
            return
        
        if not self.latest_pose.pose_detected:
            # No pose detected - maybe stop robot or hold position
            self.get_logger().warning('No pose detected - robot should hold position')
            return
        
        # Example: Access joint angles
        left_shoulder_abd = self.latest_pose.left.shoulder_abduction
        left_shoulder_ext = self.latest_pose.left.shoulder_extension
        left_elbow = self.latest_pose.left.elbow_flexion
        
        right_shoulder_abd = self.latest_pose.right.shoulder_abduction
        right_shoulder_ext = self.latest_pose.right.shoulder_extension
        right_elbow = self.latest_pose.right.elbow_flexion
        
        # TODO: Implement your robot control logic
        # Example pseudocode:
        # 
        # # Map angles to servo positions
        # left_servo_positions = self.map_angles_to_servos(
        #     left_shoulder_abd, left_shoulder_ext, left_elbow
        # )
        # right_servo_positions = self.map_angles_to_servos(
        #     right_shoulder_abd, right_shoulder_ext, right_elbow
        # )
        # 
        # # Send to robot
        # self.robot_interface.set_positions(left_servo_positions, right_servo_positions)
        
        # For now, just log occasionally
        if self.latest_pose.frame_id % 100 == 0:
            self.get_logger().info(
                f'Controlling robot with pose data from frame {self.latest_pose.frame_id}'
            )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobotControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
