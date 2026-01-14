#!/usr/bin/env python3
"""
Test camera and pose detection pipeline.

This script verifies that the camera is working and pose detection runs correctly.

Usage:
    python scripts/test_vision.py
    python scripts/test_vision.py --model models/yolov8n_pose.cvimodel
"""

import argparse
import sys
import time
from pathlib import Path

# Check if running on Milk-V (Linux with V4L2)
try:
    import subprocess
    result = subprocess.run(
        ["v4l2-ctl", "--list-devices"],
        capture_output=True,
        text=True
    )
    HAS_V4L2 = result.returncode == 0
except FileNotFoundError:
    HAS_V4L2 = False

# Try OpenCV for development testing
try:
    import cv2
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False

# Try numpy for keypoint processing
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False


def test_camera_v4l2():
    """Test camera using V4L2 (on Milk-V)."""
    print("Testing camera with V4L2...")

    # List video devices
    result = subprocess.run(
        ["v4l2-ctl", "--list-devices"],
        capture_output=True,
        text=True
    )
    print("Available video devices:")
    print(result.stdout if result.stdout else "  (none found)")

    if not result.stdout:
        print("\nNo camera detected! Check:")
        print("  1. GC2083 camera connected to J1 connector")
        print("  2. Cable orientation correct")
        print("  3. Device tree has I2C3 enabled")
        return False

    # Try to capture a frame
    device = "/dev/video0"
    print(f"\nAttempting capture from {device}...")

    # Use v4l2-ctl to capture
    result = subprocess.run(
        [
            "v4l2-ctl", "-d", device,
            "--set-fmt-video=width=640,height=480,pixelformat=YUYV",
            "--stream-mmap", "--stream-count=1",
            "--stream-to=/tmp/test_frame.raw"
        ],
        capture_output=True,
        text=True,
        timeout=5
    )

    if result.returncode == 0:
        print("Camera capture successful!")
        print(f"Frame saved to /tmp/test_frame.raw")
        return True
    else:
        print(f"Camera capture failed: {result.stderr}")
        return False


def test_camera_opencv():
    """Test camera using OpenCV (development)."""
    print("Testing camera with OpenCV...")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open camera")
        return False

    ret, frame = cap.read()
    cap.release()

    if ret:
        print(f"Captured frame: {frame.shape}")
        cv2.imwrite("/tmp/test_frame.jpg", frame)
        print("Frame saved to /tmp/test_frame.jpg")
        return True
    else:
        print("Failed to capture frame")
        return False


def test_pose_detection_mock():
    """Mock pose detection test (without actual model)."""
    print("\nTesting pose detection pipeline (mock)...")

    # Simulate keypoint output
    mock_keypoints = np.array([
        [320, 100, 0.9],   # 0: nose
        [310, 95, 0.8],    # 1: left_eye
        [330, 95, 0.8],    # 2: right_eye
        [300, 100, 0.7],   # 3: left_ear
        [340, 100, 0.7],   # 4: right_ear
        [280, 180, 0.9],   # 5: left_shoulder
        [360, 180, 0.9],   # 6: right_shoulder
        [250, 280, 0.85],  # 7: left_elbow
        [390, 280, 0.85],  # 8: right_elbow
        [230, 380, 0.8],   # 9: left_wrist
        [410, 380, 0.8],   # 10: right_wrist
        [290, 350, 0.9],   # 11: left_hip
        [350, 350, 0.9],   # 12: right_hip
        [285, 480, 0.85],  # 13: left_knee
        [355, 480, 0.85],  # 14: right_knee
        [280, 600, 0.8],   # 15: left_ankle
        [360, 600, 0.8],   # 16: right_ankle
    ])

    print(f"Mock keypoints shape: {mock_keypoints.shape}")
    print("\nKeypoint positions:")
    keypoint_names = [
        "nose", "left_eye", "right_eye", "left_ear", "right_ear",
        "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
        "left_wrist", "right_wrist", "left_hip", "right_hip",
        "left_knee", "right_knee", "left_ankle", "right_ankle"
    ]

    for i, name in enumerate(keypoint_names):
        x, y, conf = mock_keypoints[i]
        print(f"  {i:2d} {name:15s}: ({x:5.1f}, {y:5.1f}) conf={conf:.2f}")

    return True


def calculate_joint_angles(keypoints):
    """Calculate joint angles from keypoints."""
    print("\nCalculating joint angles...")

    def angle_between_points(p1, p2, p3):
        """Calculate angle at p2 formed by p1-p2-p3."""
        v1 = np.array(p1[:2]) - np.array(p2[:2])
        v2 = np.array(p3[:2]) - np.array(p2[:2])
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
        return np.degrees(np.arccos(np.clip(cos_angle, -1, 1)))

    def shoulder_pitch(shoulder, elbow):
        """Calculate shoulder pitch using atan2."""
        dx = elbow[0] - shoulder[0]
        dy = elbow[1] - shoulder[1]
        return np.degrees(np.arctan2(dy, dx))

    # Calculate angles
    angles = {}

    # Right arm
    angles["right_shoulder_pitch"] = shoulder_pitch(keypoints[6], keypoints[8])
    angles["right_elbow"] = angle_between_points(keypoints[6], keypoints[8], keypoints[10])

    # Left arm
    angles["left_shoulder_pitch"] = shoulder_pitch(keypoints[5], keypoints[7])
    angles["left_elbow"] = angle_between_points(keypoints[5], keypoints[7], keypoints[9])

    print("Joint angles:")
    for name, angle in angles.items():
        print(f"  {name:25s}: {angle:+7.2f} deg")

    return angles


def main():
    parser = argparse.ArgumentParser(description="Test vision pipeline")
    parser.add_argument(
        "--model", "-m",
        default=None,
        help="Path to cvimodel (optional)"
    )
    parser.add_argument(
        "--device", "-d",
        default="/dev/video0",
        help="Video device (default: /dev/video0)"
    )
    args = parser.parse_args()

    print("=" * 50)
    print("Shadow Control - Vision Test")
    print("=" * 50)
    print(f"V4L2 available: {HAS_V4L2}")
    print(f"OpenCV available: {HAS_OPENCV}")
    print(f"NumPy available: {HAS_NUMPY}")
    print("=" * 50)

    # Test camera
    print("\n[1/3] Camera Test")
    print("-" * 30)
    if HAS_V4L2:
        camera_ok = test_camera_v4l2()
    elif HAS_OPENCV:
        camera_ok = test_camera_opencv()
    else:
        print("No camera interface available")
        camera_ok = False

    # Test pose detection
    print("\n[2/3] Pose Detection Test")
    print("-" * 30)
    if args.model and Path(args.model).exists():
        print(f"Model: {args.model}")
        # TODO: Implement actual TDL SDK inference
        print("TDL SDK inference not yet implemented")
        pose_ok = False
    else:
        print("Using mock pose detection (no model specified)")
        pose_ok = test_pose_detection_mock()

    # Test angle calculation
    print("\n[3/3] Joint Angle Calculation")
    print("-" * 30)
    if HAS_NUMPY:
        mock_keypoints = np.array([
            [320, 100, 0.9], [310, 95, 0.8], [330, 95, 0.8],
            [300, 100, 0.7], [340, 100, 0.7], [280, 180, 0.9],
            [360, 180, 0.9], [250, 280, 0.85], [390, 280, 0.85],
            [230, 380, 0.8], [410, 380, 0.8], [290, 350, 0.9],
            [350, 350, 0.9], [285, 480, 0.85], [355, 480, 0.85],
            [280, 600, 0.8], [360, 600, 0.8]
        ])
        angles = calculate_joint_angles(mock_keypoints)
        angle_ok = len(angles) > 0
    else:
        print("NumPy not available")
        angle_ok = False

    # Summary
    print("\n" + "=" * 50)
    print("Summary")
    print("=" * 50)
    print(f"Camera:     {'PASS' if camera_ok else 'FAIL'}")
    print(f"Pose Det:   {'PASS' if pose_ok else 'FAIL'}")
    print(f"Angle Calc: {'PASS' if angle_ok else 'FAIL'}")

    if not (camera_ok and pose_ok and angle_ok):
        print("\nNext steps:")
        if not camera_ok:
            print("  - Connect GC2083 camera to J1 connector")
            print("  - Check cable orientation")
        if not pose_ok:
            print("  - Convert YOLOv8-pose model using TPU-MLIR")
            print("  - Deploy cvimodel to models/ directory")
        if not angle_ok:
            print("  - Install numpy: pip install numpy")

    sys.exit(0 if (camera_ok and pose_ok and angle_ok) else 1)


if __name__ == "__main__":
    main()
