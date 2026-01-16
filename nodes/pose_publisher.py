#!/usr/bin/env python3
"""
ROS2 Pose Estimation Publisher Node

Captures video, runs MediaPipe pose estimation, and publishes
pose data (joint angles + landmarks) to ROS2 topics.

Topics published:
  - /shadow_control/pose (shadow_control/PoseData): Complete pose data
  - /shadow_control/pose_image (sensor_msgs/Image): Annotated video frame
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

import cv2
import mediapipe as mp
import numpy as np
import math

# Import custom messages (after building the package)
from shadow_control.msg import PoseData, JointAngles, Landmark3D


class PoseEstimationNode(Node):
    """ROS2 node for pose estimation and publishing."""
    
    def __init__(self):
        super().__init__('pose_estimation_node')
        
        # Declare parameters
        self.declare_parameter('camera_id', 1)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('image_topic', '/shadow_control/pose_image')
        self.declare_parameter('pose_topic', '/shadow_control/pose')
        
        # Get parameters
        camera_id = self.get_parameter('camera_id').value
        publish_image = self.get_parameter('publish_image').value
        image_topic = self.get_parameter('image_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseData, pose_topic, 10)
        
        if publish_image:
            self.image_pub = self.create_publisher(Image, image_topic, 10)
        else:
            self.image_pub = None
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # MediaPipe pose
        self.mp_pose = mp.solutions.pose
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_styles = mp.solutions.drawing_styles
        
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        
        # Video capture
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {camera_id}')
            raise RuntimeError(f'Cannot open camera {camera_id}')
        
        self.frame_count = 0
        
        # Timer for processing frames (aim for ~30 Hz)
        self.timer = self.create_timer(0.033, self.process_frame)
        
        self.get_logger().info(f'Pose estimation node started')
        self.get_logger().info(f'  Publishing pose to: {pose_topic}')
        if publish_image:
            self.get_logger().info(f'  Publishing image to: {image_topic}')
        self.get_logger().info(f'  Camera ID: {camera_id}')
    
    def angle_between(self, v1, v2):
        """Calculate angle between two vectors in degrees."""
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-8 or n2 < 1e-8:
            return 0.0
        v1 = v1 / n1
        v2 = v2 / n2
        dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
        return float(np.degrees(np.arccos(dot)))
    
    def elbow_flexion(self, shoulder, elbow, wrist):
        """Calculate elbow flexion angle."""
        return self.angle_between(shoulder - elbow, wrist - elbow)
    
    def shoulder_abduction(self, hip, shoulder, elbow):
        """Calculate shoulder abduction angle (frontal plane)."""
        torso = shoulder - hip
        arm = elbow - shoulder
        # project to XY (image plane)
        torso = torso.copy()
        arm = arm.copy()
        torso[2] = 0
        arm[2] = 0
        return self.angle_between(torso, arm)
    
    def shoulder_extension(self, hip, shoulder, elbow):
        """Calculate shoulder extension angle (sagittal plane)."""
        torso = shoulder - hip
        arm = elbow - shoulder
        # project to YZ (sagittal-like)
        torso = torso.copy()
        arm = arm.copy()
        torso[0] = 0
        arm[0] = 0
        return self.angle_between(torso, arm)
    
    def process_frame(self):
        """Process one frame and publish results."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to read frame from camera')
            return
        
        h, w = frame.shape[:2]
        
        # Run pose estimation
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)
        
        # Create ROS message header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_frame'
        
        # Create pose message
        pose_msg = PoseData()
        pose_msg.header = header
        pose_msg.frame_id = self.frame_count
        
        if results.pose_landmarks:
            lm = results.pose_landmarks.landmark
            
            # Draw skeleton on frame
            if self.image_pub:
                self.mp_draw.draw_landmarks(
                    frame,
                    results.pose_landmarks,
                    self.mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=self.mp_styles.get_default_pose_landmarks_style(),
                )
            
            def p(i):
                """Get 3D position of landmark."""
                return np.array([lm[i].x, lm[i].y, lm[i].z], dtype=np.float32)
            
            # Extract key landmarks
            L_SH, L_EL, L_WR, L_HIP = p(11), p(13), p(15), p(23)
            R_SH, R_EL, R_WR, R_HIP = p(12), p(14), p(16), p(24)
            
            # Calculate angles
            left_elbow = self.elbow_flexion(L_SH, L_EL, L_WR)
            right_elbow = self.elbow_flexion(R_SH, R_EL, R_WR)
            
            left_abd = self.shoulder_abduction(L_HIP, L_SH, L_EL)
            right_abd = self.shoulder_abduction(R_HIP, R_SH, R_EL)
            
            left_ext = self.shoulder_extension(L_HIP, L_SH, L_EL)
            right_ext = self.shoulder_extension(R_HIP, R_SH, R_EL)
            
            # Fill joint angles
            pose_msg.left = JointAngles()
            pose_msg.left.shoulder_abduction = left_abd
            pose_msg.left.shoulder_extension = left_ext
            pose_msg.left.elbow_flexion = left_elbow
            
            pose_msg.right = JointAngles()
            pose_msg.right.shoulder_abduction = right_abd
            pose_msg.right.shoulder_extension = right_ext
            pose_msg.right.elbow_flexion = right_elbow
            
            # Fill landmarks
            def make_landmark(pos, idx):
                """Create Landmark3D message."""
                lmk = Landmark3D()
                lmk.x = float(pos[0])
                lmk.y = float(pos[1])
                lmk.z = float(pos[2])
                lmk.visibility = float(lm[idx].visibility)
                return lmk
            
            pose_msg.left_shoulder = make_landmark(L_SH, 11)
            pose_msg.left_elbow = make_landmark(L_EL, 13)
            pose_msg.left_wrist = make_landmark(L_WR, 15)
            pose_msg.left_hip = make_landmark(L_HIP, 23)
            
            pose_msg.right_shoulder = make_landmark(R_SH, 12)
            pose_msg.right_elbow = make_landmark(R_EL, 14)
            pose_msg.right_wrist = make_landmark(R_WR, 16)
            pose_msg.right_hip = make_landmark(R_HIP, 24)
            
            pose_msg.detection_confidence = 1.0
            pose_msg.pose_detected = True
            
            # Add text overlay
            if self.image_pub:
                self.add_text_overlay(frame, left_abd, left_ext, left_elbow,
                                     right_abd, right_ext, right_elbow)
        else:
            # No pose detected
            pose_msg.pose_detected = False
            pose_msg.detection_confidence = 0.0
            
            # Initialize empty joint angles
            pose_msg.left = JointAngles()
            pose_msg.right = JointAngles()
            
            # Initialize empty landmarks
            pose_msg.left_shoulder = Landmark3D()
            pose_msg.left_elbow = Landmark3D()
            pose_msg.left_wrist = Landmark3D()
            pose_msg.left_hip = Landmark3D()
            pose_msg.right_shoulder = Landmark3D()
            pose_msg.right_elbow = Landmark3D()
            pose_msg.right_wrist = Landmark3D()
            pose_msg.right_hip = Landmark3D()
        
        # Publish pose data
        self.pose_pub.publish(pose_msg)
        
        # Publish annotated image
        if self.image_pub:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header = header
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish image: {e}')
        
        self.frame_count += 1
    
    def add_text_overlay(self, frame, left_abd, left_ext, left_elbow,
                         right_abd, right_ext, right_elbow):
        """Add text overlay to frame."""
        def put_text_with_bg(img, text, org, font_scale=0.55, thickness=1, pad=4):
            font = cv2.FONT_HERSHEY_SIMPLEX
            (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
            x, y = org
            cv2.rectangle(img, (x - pad, y - th - pad),
                         (x + tw + pad, y + baseline + pad), (0, 0, 0), -1)
            cv2.putText(img, text, (x, y), font, font_scale,
                       (255, 255, 255), thickness, cv2.LINE_AA)
        
        hud_lines = [
            "Angles (deg) - ROS2 Publishing",
            f"L Shoulder Abd: {left_abd:5.1f}   R: {right_abd:5.1f}",
            f"L Shoulder Ext: {left_ext:5.1f}   R: {right_ext:5.1f}",
            f"L Elbow Flex:   {left_elbow:5.1f}   R: {right_elbow:5.1f}",
            f"Frame: {self.frame_count}",
        ]
        
        x0, y0 = 12, 24
        for i, line in enumerate(hud_lines):
            put_text_with_bg(frame, line, (x0, y0 + i * 22), font_scale=0.55, thickness=1)
    
    def destroy_node(self):
        """Clean up resources."""
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PoseEstimationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
