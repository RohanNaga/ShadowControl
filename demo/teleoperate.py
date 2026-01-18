#!/usr/bin/env python3
"""
Shadow Control - Real-time Teleoperation

Captures pose from webcam using MediaPipe and commands arm servos in real-time.

Usage:
    python demo/teleoperate.py                    # Run with defaults
    python demo/teleoperate.py --camera 0         # Use camera index 0
    python demo/teleoperate.py --port /dev/ttyUSB0  # Specify serial port
    python demo/teleoperate.py --no-servo         # Pose visualization only
    python demo/teleoperate.py --smoothing 0.3    # Adjust smoothing (0-1)

Controls:
    ESC - Quit
    SPACE - Toggle servo control (pause/resume)
    R - Reset servos to neutral (0 degrees)
"""

import argparse
import csv
import json
import logging
import os
import sys
import time
import urllib.request
from datetime import datetime

import cv2

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)
import mediapipe as mp
from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision
import numpy as np

# Add parent directory for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ============================================================================
# Configuration
# ============================================================================

# Servo IDs and their pose angle mapping
# Format: servo_id -> (pose_angle_name, scale, offset, min_angle, max_angle)
#
# Mapping (puppeteer style - same side):
#   Shoulder Extension -> Servo 5/8 (Shoulder1) - forward/back movement
#   Shoulder Abduction -> Servo 6/9 (Shoulder2) - side movement
#   Elbow Flexion      -> Servo 7/10 (Elbow)    - arm bend

SERVO_MAPPING = {
    # Scale: 1.0 = same direction as MediaPipe, -1.0 = inverted
    #
    # Shoulder1 (Extension): forward/back movement in YZ plane
    # Shoulder2 (Abduction): side-to-side movement in XY plane
    # Elbow: arm bend (0° straight, 90° bent)
    #
    # PUPPETEER MODE: same side control (your right = robot right)
    # Right side servos are inverted to match robot's coordinate system
    5: {"name": "R_Shoulder1", "pose_key": "right_ext", "scale": -1.0},
    6: {"name": "R_Shoulder2", "pose_key": "right_abd", "scale": -1.0},
    7: {"name": "R_Elbow", "pose_key": "right_elbow", "scale": 1.0},  # changed back to be 1.0 scale
    8: {"name": "L_Shoulder1", "pose_key": "left_ext", "scale": 1.0},
    9: {"name": "L_Shoulder2", "pose_key": "left_abd", "scale": 1.0},
    10: {"name": "L_Elbow", "pose_key": "left_elbow", "scale": 1.0},
}

# Neutral positions from calibration - used for reset (12-bit range: 0-4095)
# All servos calibrated to middle (2048) for full range in both directions
NEUTRAL_POSITIONS = {
    5: 2048,
    6: 2048,
    7: 2048,
    8: 2048,
    9: 2048,
    10: 2048,
}

CALIBRATION_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "servo_calibration.json",
)

# Model file path and URL
# Options: pose_landmarker_lite (fast, less accurate), pose_landmarker_full (balanced), pose_landmarker_heavy (accurate, slow)
MODEL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models")
MODEL_FILE = os.path.join(MODEL_DIR, "pose_landmarker_heavy.task")
MODEL_URL = "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task"

STEPS_PER_REV = 4096
BAUDRATE = 500000

# Pose landmark indices (same as old API)
# https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
POSE_LANDMARKS = {
    "left_shoulder": 11,
    "right_shoulder": 12,
    "left_elbow": 13,
    "right_elbow": 14,
    "left_wrist": 15,
    "right_wrist": 16,
    "left_hip": 23,
    "right_hip": 24,
}

# Pose connections for drawing skeleton
POSE_CONNECTIONS = [
    (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),  # Arms
    (11, 23), (12, 24), (23, 24),  # Torso
    (23, 25), (25, 27), (24, 26), (26, 28),  # Legs
]


def download_model():
    """Download pose landmarker model if not present."""
    if os.path.exists(MODEL_FILE):
        return MODEL_FILE

    print(f"Downloading pose landmarker model...")
    os.makedirs(MODEL_DIR, exist_ok=True)
    urllib.request.urlretrieve(MODEL_URL, MODEL_FILE)
    print(f"Model saved to {MODEL_FILE}")
    return MODEL_FILE


def angle_between(v1, v2):
    """Calculate angle between two vectors in degrees."""
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-8 or n2 < 1e-8:
        return float("nan")
    v1 = v1 / n1
    v2 = v2 / n2
    dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
    return float(np.degrees(np.arccos(dot)))


def elbow_flexion(shoulder, elbow, wrist):
    """
    Calculate elbow flexion angle using plane projection for improved accuracy.
    Returns 0° for straight arm, increases as elbow bends (0-180° range).
    
    Method: Projects vectors onto plane perpendicular to upper arm axis.
    This removes the effect of shoulder roll/rotation, giving pure flexion angle.
    Similar structure to shoulder_abduction and shoulder_extension for consistency.
    
    Coordinate System:
    - Upper arm vector: shoulder → elbow
    - Forearm vector: elbow → wrist
    - Projects to plane perpendicular to upper arm axis (similar to shoulder_roll)
    - Measures angle between upper arm direction and forearm in that plane
    - This gives flexion angle independent of shoulder roll component
    
    GIMBAL LOCK: When forearm is nearly parallel to upper arm, the projection
    becomes very small and the angle becomes unreliable. We handle edge cases.
    """
    upper_arm = elbow - shoulder  # Vector from shoulder to elbow
    forearm = wrist - elbow       # Vector from elbow to wrist
    
    # Keep full vectors for comparison
    upper_arm_full = upper_arm.copy()
    forearm_full = forearm.copy()
    
    # Get full vector lengths for comparison
    upper_arm_full_norm = np.linalg.norm(upper_arm_full)
    forearm_full_norm = np.linalg.norm(forearm_full)
    if upper_arm_full_norm < 1e-6 or forearm_full_norm < 1e-6:
        return float("nan")
    
    # Normalize vectors
    upper_arm_unit = upper_arm / upper_arm_full_norm
    forearm_unit = forearm / forearm_full_norm
    
    # Project forearm onto plane perpendicular to upper arm axis
    # Remove component parallel to upper arm (this removes roll component)
    forearm_perp = forearm_unit - np.dot(forearm_unit, upper_arm_unit) * upper_arm_unit
    
    # Check if projected forearm vector is valid
    forearm_perp_norm = np.linalg.norm(forearm_perp)
    
    # GIMBAL LOCK FIX: If the perpendicular projection is less than 30% of the full forearm length,
    # the forearm is mostly parallel to the upper arm and the angle is unreliable
    projection_ratio = forearm_perp_norm / forearm_full_norm if forearm_full_norm > 1e-6 else 0.0
    if projection_ratio < 0.3:
        # Forearm is nearly parallel to upper arm - check if extended or bent
        dot_parallel = np.dot(forearm_unit, upper_arm_unit)
        if dot_parallel > 0.9:
            return 0.0  # Fully extended (same direction)
        elif dot_parallel < -0.9:
            return 180.0  # Fully bent (opposite direction)
        else:
            return float("nan")  # Angle is unreliable in gimbal lock region
    
    # Reference direction: upper arm direction (reversed, from elbow back to shoulder)
    # This is the direction we measure flexion relative to
    upper_arm_reversed = -upper_arm_unit
    
    # Project the reversed upper arm onto the perpendicular plane
    # (should already be in plane since it's opposite to upper_arm_unit)
    upper_arm_rev_perp = upper_arm_reversed - np.dot(upper_arm_reversed, upper_arm_unit) * upper_arm_unit
    upper_arm_rev_perp_norm = np.linalg.norm(upper_arm_rev_perp)
    
    if upper_arm_rev_perp_norm < 1e-6:
        # Edge case: shouldn't happen normally
        return float("nan")
    
    # Normalize perpendicular components
    forearm_perp = forearm_perp / forearm_perp_norm
    upper_arm_rev_perp = upper_arm_rev_perp / upper_arm_rev_perp_norm
    
    # Calculate angle from upper arm direction (reversed) to forearm direction
    # This gives us the flexion angle: 0° when straight, 180° when fully bent
    dot = np.clip(np.dot(upper_arm_rev_perp, forearm_perp), -1.0, 1.0)
    angle = np.degrees(np.arccos(dot))
    
    # The angle gives us flexion:
    # - 0° = vectors aligned (straight arm) -> 0° flexion
    # - 90° = vectors perpendicular -> 90° flexion
    # - 180° = vectors opposite (fully bent) -> 180° flexion
    return min(180.0, max(0.0, angle))


def shoulder_roll(shoulder, elbow, wrist):
    """
    Calculate shoulder roll (external rotation around upper arm axis).
    Returns angle in range 0° to 180°:
    - 0° = forearm pointing down (internal rotation)
    - 90° = forearm horizontal (neutral rotation)
    - 180° = forearm pointing up (external rotation)
    
    Coordinate System:
    - Measures rotation of forearm around the upper arm axis (shoulder->elbow)
    - Based on "External Rotation" diagram where 0° = down, 180° = up
    - Calculates the angle between the forearm and a reference direction (down)
      projected onto the plane perpendicular to the upper arm
    """
    upper_arm = elbow - shoulder  # Upper arm vector
    forearm = wrist - elbow       # Forearm vector
    
    # Normalize vectors
    upper_arm_norm = np.linalg.norm(upper_arm)
    forearm_norm = np.linalg.norm(forearm)
    if upper_arm_norm < 1e-6 or forearm_norm < 1e-6:
        return float("nan")
    
    upper_arm_unit = upper_arm / upper_arm_norm
    forearm_unit = forearm / forearm_norm
    
    # Reference direction: "down" in MediaPipe coordinates (positive Y)
    down_ref = np.array([0.0, 1.0, 0.0])
    
    # Project both forearm and reference onto plane perpendicular to upper arm
    # Remove component parallel to upper arm
    forearm_perp = forearm_unit - np.dot(forearm_unit, upper_arm_unit) * upper_arm_unit
    down_ref_perp = down_ref - np.dot(down_ref, upper_arm_unit) * upper_arm_unit
    
    # Normalize perpendicular components
    forearm_perp_norm = np.linalg.norm(forearm_perp)
    down_ref_perp_norm = np.linalg.norm(down_ref_perp)
    
    if forearm_perp_norm < 1e-6 or down_ref_perp_norm < 1e-6:
        return float("nan")  # Invalid projection
    
    forearm_perp = forearm_perp / forearm_perp_norm
    down_ref_perp = down_ref_perp / down_ref_perp_norm
    
    # Calculate angle between reference and forearm in perpendicular plane
    dot = np.clip(np.dot(down_ref_perp, forearm_perp), -1.0, 1.0)
    angle = np.degrees(np.arccos(dot))
    
    # Use cross product to determine rotation direction
    cross = np.cross(down_ref_perp, forearm_perp)
    cross_along_arm = np.dot(cross, upper_arm_unit)
    
    # Map to 0-180 range based on rotation direction
    if cross_along_arm < 0:
        angle = 360.0 - angle
    
    # Normalize to 0-180 range
    if angle > 180.0:
        angle = 360.0 - angle
    
    return min(180.0, max(0.0, angle))


def shoulder_abduction(hip, shoulder, elbow):
    """
    Calculate shoulder abduction (horizontal plane movement, like "Horizontal Adduction" diagram).
    Returns angle in range 0° to 180°, or NaN if unreliable (gimbal lock region):
    - 0° = arm at side (aligned with torso vertically, no horizontal deviation)
    - 90° = arm horizontal out to side (perpendicular to torso)
    - 180° = arm across body (maximum horizontal adduction)

    Coordinate System (XY/horizontal plane):
    - Measures angle from vertical reference (downward) to arm direction in horizontal plane
    - Based on "Horizontal Adduction" diagram where 0° = arms at side, 90° = horizontal, 180° = across body

    GIMBAL LOCK: When arm is extended ~90° forward/back, the XY projection
    becomes very small and the angle becomes unreliable. We return NaN in this case.
    """
    arm = elbow - shoulder
    arm_full = arm.copy()
    arm = arm.copy()

    # Get full arm length for comparison
    arm_full_norm = np.linalg.norm(arm_full)
    if arm_full_norm < 1e-6:
        return float("nan")

    # Project to XY plane (horizontal plane, removes depth/z component)
    arm[2] = 0

    # Check if arm vector is valid
    arm_norm = np.linalg.norm(arm)

    # GIMBAL LOCK FIX: If the XY projection is less than 30% of the full arm length,
    # the arm is mostly pointing forward/back and this angle is unreliable
    projection_ratio = arm_norm / arm_full_norm
    if projection_ratio < 0.3:
        return float("nan")  # Angle is unreliable in gimbal lock region

    # Reference direction: downward in XY plane (positive Y direction in MediaPipe)
    # In MediaPipe, Y increases downward, so [0, 1, 0] is "down"
    down_ref = np.array([0.0, 1.0, 0.0])

    # Normalize arm vector in XY plane
    arm_xy = arm / arm_norm

    # Calculate angle from vertical (down) to arm direction
    # This gives us the abduction angle: 0° when arm is down, 90° when horizontal
    dot = np.clip(np.dot(down_ref, arm_xy), -1.0, 1.0)
    angle = np.degrees(np.arccos(dot))

    # angle_between returns 0-180°, which is what we want
    # 0° = arm pointing down (at side)
    # 90° = arm pointing sideways (horizontal)
    # 180° = arm pointing up (across body)
    return min(180.0, max(0.0, angle))


def shoulder_extension(hip, shoulder, elbow):
    """
    Calculate shoulder extension (vertical elevation in sagittal/YZ plane).
    Returns angle in range 0° to 180°, or NaN if unreliable (gimbal lock region):
    - 0° = arm down at side (aligned with torso vertically)
    - 90° = arm horizontal forward (perpendicular to torso)
    - 180° = arm straight up overhead (opposite to torso direction)

    GIMBAL LOCK: When arm is abducted ~90° (pointing sideways), the YZ projection
    becomes very small and the angle becomes unreliable. We return NaN in this case.
    """
    arm = elbow - shoulder   # Vector from shoulder to elbow
    arm_full = arm.copy()
    arm = arm.copy()

    # Get full arm length for comparison
    arm_full_norm = np.linalg.norm(arm_full)
    if arm_full_norm < 1e-6:
        return float("nan")

    # Project to YZ plane (sagittal plane, removes side/x component)
    arm[0] = 0

    # Check if projected arm vector is valid
    arm_norm = np.linalg.norm(arm)

    # GIMBAL LOCK FIX: If the YZ projection is less than 30% of the full arm length,
    # the arm is mostly pointing sideways and this angle is unreliable
    projection_ratio = arm_norm / arm_full_norm
    if projection_ratio < 0.3:
        return float("nan")  # Angle is unreliable in gimbal lock region

    # Reference direction: downward in YZ plane (positive Y direction in MediaPipe)
    down_ref = np.array([0.0, 1.0, 0.0])

    # Normalize arm vector in YZ plane
    arm_yz = arm / arm_norm

    # Calculate angle from vertical (down) to arm direction in YZ plane
    dot = np.clip(np.dot(down_ref, arm_yz), -1.0, 1.0)
    angle = np.degrees(np.arccos(dot))

    return min(180.0, max(0.0, angle))


def get_pose_angles(landmarks):
    """
    Extract joint angles from MediaPipe pose landmarks.

    Args:
        landmarks: List of NormalizedLandmark from PoseLandmarker result

    Returns dict with keys:
        left_abd, right_abd - shoulder abduction (0-180°)
        left_ext, right_ext - shoulder extension (0-180°)
        left_roll, right_roll - shoulder roll/external rotation (0-180°)
        left_elbow, right_elbow - elbow flexion (0-180°)
    """
    def p(i):
        lm = landmarks[i]
        return np.array([lm.x, lm.y, lm.z], dtype=np.float32)

    # Extract key landmarks
    L_SH, L_EL, L_WR, L_HIP = p(11), p(13), p(15), p(23)
    R_SH, R_EL, R_WR, R_HIP = p(12), p(14), p(16), p(24)

    return {
        "left_elbow": elbow_flexion(L_SH, L_EL, L_WR),
        "right_elbow": elbow_flexion(R_SH, R_EL, R_WR),
        "left_abd": shoulder_abduction(L_HIP, L_SH, L_EL),
        "right_abd": shoulder_abduction(R_HIP, R_SH, R_EL),
        "left_ext": shoulder_extension(L_HIP, L_SH, L_EL),
        "right_ext": shoulder_extension(R_HIP, R_SH, R_EL),
        "left_roll": shoulder_roll(L_SH, L_EL, L_WR),
        "right_roll": shoulder_roll(R_SH, R_EL, R_WR),
    }


def draw_angle_arc(frame, p1_px, p2_px, p3_px, angle_deg, color=(255, 0, 255), alpha=0.4, radius=None):
    """
    Draw an angle arc between three points (p2 is the vertex/joint).
    
    Args:
        frame: OpenCV image frame
        p1_px: (x, y) pixel coordinates of first point
        p2_px: (x, y) pixel coordinates of joint/vertex (where angle is measured)
        p3_px: (x, y) pixel coordinates of third point
        angle_deg: Angle in degrees to display
        color: BGR color tuple for the arc
        alpha: Transparency of filled arc (0-1)
        radius: Optional radius in pixels (auto-calculated if None)
    
    Returns:
        frame with arc drawn
    """
    if np.isnan(angle_deg):
        return frame
    
    x1, y1 = p1_px
    x2, y2 = p2_px
    x3, y3 = p3_px
    
    # Calculate vectors from joint (p2) to each point
    a = np.array([x1 - x2, y1 - y2], dtype=float)
    b = np.array([x3 - x2, y3 - y2], dtype=float)
    
    # Prevent division by zero
    if np.linalg.norm(a) < 1e-6 or np.linalg.norm(b) < 1e-6:
        return frame
    
    # Auto-calculate radius if not provided
    if radius is None:
        base_radius = int(min(max(np.linalg.norm(a) * 0.15, 15), 50))
    else:
        base_radius = radius
    
    # Normalize vectors
    a_unit = a / np.linalg.norm(a)
    b_unit = b / np.linalg.norm(b)
    
    # Generate arc points between the two vectors
    num_points = 30
    arc_points = []
    for i in range(num_points + 1):
        t = i / num_points
        # Interpolate between unit vectors
        vec = (1 - t) * a_unit + t * b_unit
        if np.linalg.norm(vec) < 1e-6:
            continue
        vec = vec / np.linalg.norm(vec)
        px = int(x2 + vec[0] * base_radius)
        py = int(y2 + vec[1] * base_radius)
        arc_points.append((px, py))
    
    if len(arc_points) < 2:
        return frame
    
    # Draw filled arc (semi-transparent)
    poly = [(x2, y2)] + arc_points + [(x2, y2)]
    overlay = frame.copy()
    cv2.fillPoly(overlay, [np.array(poly, dtype=np.int32)], color)
    frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)
    
    # Draw arc outline
    cv2.polylines(frame, [np.array(arc_points, dtype=np.int32)], False, color, 2)
    
    # Display angle value near the arc
    text = f"{int(angle_deg)}°"
    text_x = x2 - 30
    text_y = y2 - base_radius - 10
    
    # Draw text with background for readability
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    thickness = 2
    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    cv2.rectangle(frame, 
                  (text_x - 3, text_y - text_height - 3),
                  (text_x + text_width + 3, text_y + baseline + 3),
                  (0, 0, 0), -1)
    cv2.putText(frame, text, (text_x, text_y), font, font_scale, color, thickness, cv2.LINE_AA)
    
    return frame


def draw_landmarks(frame, landmarks):
    """Draw pose landmarks and connections on frame."""
    h, w = frame.shape[:2]

    # Draw connections
    for start_idx, end_idx in POSE_CONNECTIONS:
        if start_idx < len(landmarks) and end_idx < len(landmarks):
            start = landmarks[start_idx]
            end = landmarks[end_idx]
            if start.visibility > 0.5 and end.visibility > 0.5:
            start_point = (int(start.x * w), int(start.y * h))
            end_point = (int(end.x * w), int(end.y * h))
            cv2.line(frame, start_point, end_point, (0, 255, 0), 2)

    # Draw landmarks
    for i, lm in enumerate(landmarks):
        if lm.visibility > 0.5:
        x, y = int(lm.x * w), int(lm.y * h)
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)


# ============================================================================
# Servo Control (imported from src.servo_controller)
# ============================================================================

from src.servo_controller import ServoController, auto_detect_port as _auto_detect_port, configure_logging


# ============================================================================
# Angle Smoother
# ============================================================================


class AngleSmoother:
    """Exponential moving average smoother for angles."""

    def __init__(self, alpha=0.3):
        """
        Args:
            alpha: Smoothing factor (0-1). Higher = more responsive, lower = smoother.
        """
        self.alpha = alpha
        self.values = {}

    def smooth(self, key, new_value):
        """Apply exponential smoothing to a value."""
        if np.isnan(new_value):
            return self.values.get(key, 0.0)

        if key not in self.values:
            self.values[key] = new_value
        else:
            self.values[key] = self.alpha * new_value + (1 - self.alpha) * self.values[key]

        return self.values[key]

    def smooth_dict(self, values_dict):
        """Smooth all values in a dictionary."""
        return {k: self.smooth(k, v) for k, v in values_dict.items()}


# ============================================================================
# Visualization
# ============================================================================


def put_text_with_bg(img, text, org, font_scale=0.7, thickness=2, pad=6, color=(255, 255, 255)):
    """Draw text with background for readability."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = org
    cv2.rectangle(img, (x - pad, y - th - pad), (x + tw + pad, y + baseline + pad), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)


def draw_hud(frame, pose_angles, servo_angles, servo_enabled, fps, encoder_angles=None, raw_positions=None, temperatures=None):
    """Draw heads-up display with angle info."""
    if encoder_angles is None:
        encoder_angles = {}
    if raw_positions is None:
        raw_positions = {}
    if temperatures is None:
        temperatures = {}
    h, w = frame.shape[:2]

    # Status bar (top)
    status_color = (0, 255, 0) if servo_enabled else (0, 0, 255)
    status_text = "SERVOS: ON" if servo_enabled else "SERVOS: OFF"
    put_text_with_bg(frame, f"{status_text}  |  FPS: {fps:.1f}", (15, 35), font_scale=0.8, thickness=2, color=status_color)

    # Left column: Pose angles from MediaPipe (relative to first frame)
    y = 80
    put_text_with_bg(frame, "POSE (rel)", (15, y), font_scale=0.7, color=(0, 255, 255))
    y += 35
    for key in ["left_abd", "right_abd"]:  # Only show active ones
        val = pose_angles.get(key, float("nan"))
        label = key.replace("left_", "L_").replace("right_", "R_")
        text = f"{label}: {val:+6.1f}" if not np.isnan(val) else f"{label}:   N/A"
        put_text_with_bg(frame, text, (15, y), font_scale=0.65)
        y += 30

    # Middle column: Servo commands (what we're sending)
    y = 80
    col2_x = w // 3
    put_text_with_bg(frame, "CMD (deg)", (col2_x, y), font_scale=0.7, color=(255, 255, 0))
    y += 35
    for servo_id, config in SERVO_MAPPING.items():
        angle = servo_angles.get(servo_id, 0)
        name = config['name'].replace("_Shoulder", "_Sh").replace("_Elbow", "_El")
        text = f"{name}: {angle:+6.1f}"
        put_text_with_bg(frame, text, (col2_x, y), font_scale=0.65)
        y += 30

    # Right column: Encoder readings (actual angle + raw position)
    y = 80
    col3_x = 2 * w // 3
    put_text_with_bg(frame, "ENC (deg/raw)", (col3_x, y), font_scale=0.7, color=(0, 255, 0))
    y += 35
    for servo_id, config in SERVO_MAPPING.items():
        enc = encoder_angles.get(servo_id)
        raw = raw_positions.get(servo_id)
        temp = temperatures.get(servo_id)
        name = config['name'].replace("_Shoulder", "_Sh").replace("_Elbow", "_El")

        # Determine temperature color (green < 50C, yellow < 60C, red >= 60C)
        temp_str = ""
        temp_color = (0, 255, 0)
        if temp is not None:
            temp_str = f" {temp}C"
            if temp >= 60:
                temp_color = (0, 0, 255)  # Red
            elif temp >= 50:
                temp_color = (0, 165, 255)  # Orange

        if enc is not None and raw is not None:
            text = f"{name}: {enc:+6.1f} ({raw})"
            put_text_with_bg(frame, text, (col3_x, y), font_scale=0.55, color=(0, 255, 0))
        elif raw is not None:
            text = f"{name}: --- ({raw})"
            put_text_with_bg(frame, text, (col3_x, y), font_scale=0.55, color=(255, 165, 0))
        else:
            text = f"{name}: ---"
            put_text_with_bg(frame, text, (col3_x, y), font_scale=0.55, color=(128, 128, 128))

        # Show temperature next to encoder reading
        if temp_str:
            put_text_with_bg(frame, temp_str, (col3_x + 200, y), font_scale=0.5, color=temp_color)
        y += 30

    # Controls hint (bottom)
    put_text_with_bg(frame, "ESC:quit  SPACE:toggle  R:reset  C:calibrate", (15, h - 15), font_scale=0.6)


# ============================================================================
# Data Logger
# ============================================================================

ANNOTATION_LABELS = {
    ord("1"): "arm_forward",
    ord("2"): "arm_side",
    ord("3"): "elbow_bend",
    ord("4"): "both_arms",
    ord("5"): "neutral",
    ord("0"): "other",
}


class DataLogger:
    """Log pose angles, servo commands, and annotations in human-readable table format."""

    def __init__(self, output_dir="logs"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_id = timestamp
        self.filepath = os.path.join(output_dir, f"teleop_log_{timestamp}.txt")
        self.csv_filepath = os.path.join(output_dir, f"teleop_log_{timestamp}.csv")
        self.frames_dir = os.path.join(output_dir, f"frames_{timestamp}")
        os.makedirs(self.frames_dir, exist_ok=True)

        # Human-readable table file
        self.file = open(self.filepath, "w")
        self._write_header()

        # CSV file for analysis
        self.csv_file = open(self.csv_filepath, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "frame", "L_ext", "L_abd", "L_elb", "R_ext", "R_abd", "R_elb",
            "cmd_6", "cmd_7", "cmd_9", "cmd_10",
            "enc_6", "enc_7", "enc_9", "enc_10", "annotation"
        ])

        self.current_annotation = ""
        self.frame_count = 0

        # Frame capture state
        self.capture_pending = False
        self.capture_label = ""
        self.capture_start_time = 0
        self.capture_delay_ms = 500
        self.capture_interval_ms = 300
        self.capture_count = 5
        self.captured_frames = 0
        self.last_capture_time = 0
        self.capture_sequence = 0

        print(f"Logging to: {self.filepath}")
        print(f"CSV data:   {self.csv_filepath}")

    def _write_header(self):
        """Write table header."""
        self.file.write("=" * 140 + "\n")
        self.file.write(f"  Shadow Control - Teleoperation Log - {self.session_id}\n")
        self.file.write("=" * 140 + "\n\n")
        self.file.write("  POSE ANGLES (MediaPipe)              │  SERVO COMMANDS (sent)              │  ENCODER (actual)\n")
        self.file.write("─" * 140 + "\n")
        header = (
            f"{'Frame':>6} │ "
            f"{'L_ext':>6} {'L_abd':>6} {'L_elb':>6} {'R_ext':>6} {'R_abd':>6} {'R_elb':>6} │ "
            f"{'cmd6':>6} {'cmd7':>6} {'cmd9':>6} {'cmd10':>6} │ "
            f"{'enc6':>6} {'enc7':>6} {'enc9':>6} {'enc10':>6} │ "
            f"{'Note':<12}"
        )
        self.file.write(header + "\n")
        self.file.write("─" * 140 + "\n")

    def _fmt(self, val, width=6):
        """Format a value for display."""
        if val is None or (isinstance(val, float) and np.isnan(val)):
            return " " * (width - 1) + "-"
        return f"{val:>{width}.1f}"

    def log(self, pose_angles, servo_angles, encoder_angles=None):
        """Log a frame of data."""
        if encoder_angles is None:
            encoder_angles = {}
        self.frame_count += 1

        # Get pose values
        l_ext = pose_angles.get('left_ext', 0)
        l_abd = pose_angles.get('left_abd', 0)
        l_elb = pose_angles.get('left_elbow', 0)
        r_ext = pose_angles.get('right_ext', 0)
        r_abd = pose_angles.get('right_abd', 0)
        r_elb = pose_angles.get('right_elbow', 0)

        # Get servo commands (6, 7, 9, 10 are active)
        cmd6 = servo_angles.get(6, 0)
        cmd7 = servo_angles.get(7, 0)
        cmd9 = servo_angles.get(9, 0)
        cmd10 = servo_angles.get(10, 0)

        # Get encoder readings
        enc6 = encoder_angles.get(6)
        enc7 = encoder_angles.get(7)
        enc9 = encoder_angles.get(9)
        enc10 = encoder_angles.get(10)

        # Human-readable table row
        line = (
            f"{self.frame_count:>6} │ "
            f"{self._fmt(l_ext)} {self._fmt(l_abd)} {self._fmt(l_elb)} {self._fmt(r_ext)} {self._fmt(r_abd)} {self._fmt(r_elb)} │ "
            f"{self._fmt(cmd6)} {self._fmt(cmd7)} {self._fmt(cmd9)} {self._fmt(cmd10)} │ "
            f"{self._fmt(enc6)} {self._fmt(enc7)} {self._fmt(enc9)} {self._fmt(enc10)} │ "
            f"{self.current_annotation:<12}"
        )
        self.file.write(line + "\n")

        # CSV row for analysis
        self.csv_writer.writerow([
            self.frame_count,
            f"{l_ext:.1f}", f"{l_abd:.1f}", f"{l_elb:.1f}", f"{r_ext:.1f}", f"{r_abd:.1f}", f"{r_elb:.1f}",
            f"{cmd6:.1f}", f"{cmd7:.1f}", f"{cmd9:.1f}", f"{cmd10:.1f}",
            f"{enc6:.1f}" if enc6 is not None else "",
            f"{enc7:.1f}" if enc7 is not None else "",
            f"{enc9:.1f}" if enc9 is not None else "",
            f"{enc10:.1f}" if enc10 is not None else "",
            self.current_annotation
        ])

        # Clear annotation after logging (one-shot)
        annotation_logged = self.current_annotation
        self.current_annotation = ""
        return annotation_logged

    def set_annotation(self, label):
        """Set annotation for next logged frame and start frame capture."""
        self.current_annotation = label
        print(f"ANNOTATION: {label} - capturing frames in 1 second...")

        # Start frame capture sequence
        self.capture_pending = True
        self.capture_label = label
        self.capture_start_time = time.time() * 1000
        self.captured_frames = 0
        self.capture_sequence += 1

    def check_capture(self, frame, pose_angles, servo_angles):
        """Check if we should capture a frame. Call this every frame."""
        if not self.capture_pending:
            return

        now_ms = time.time() * 1000
        elapsed = now_ms - self.capture_start_time

        # Wait for delay before starting capture
        if elapsed < self.capture_delay_ms:
            return

        # Check if it's time for next capture
        capture_elapsed = elapsed - self.capture_delay_ms
        expected_captures = int(capture_elapsed / self.capture_interval_ms) + 1

        if expected_captures > self.captured_frames and self.captured_frames < self.capture_count:
            # Save this frame
            self.captured_frames += 1
            filename = f"{self.capture_label}_seq{self.capture_sequence:03d}_f{self.captured_frames}.jpg"
            filepath = os.path.join(self.frames_dir, filename)
            cv2.imwrite(filepath, frame)

            # Also save a text file with the angles
            angles_file = filepath.replace('.jpg', '.txt')
            with open(angles_file, 'w') as f:
                f.write(f"Frame: {self.frame_count}\n")
                f.write(f"Annotation: {self.capture_label}\n")
                f.write(f"Capture: {self.captured_frames}/{self.capture_count}\n\n")
                f.write("Pose Angles:\n")
                for k, v in pose_angles.items():
                    f.write(f"  {k}: {v:.2f}\n")
                f.write("\nServo Commands:\n")
                for k, v in servo_angles.items():
                    f.write(f"  servo_{k}: {v:.2f}\n")

            print(f"  Captured frame {self.captured_frames}/{self.capture_count}: {filename}")

            if self.captured_frames >= self.capture_count:
                self.capture_pending = False
                print(f"  Capture complete for '{self.capture_label}'")

    def close(self):
        """Close log files and write summary."""
        # Write footer to table file
        self.file.write("─" * 100 + "\n")
        self.file.write(f"  Total frames: {self.frame_count}\n")
        self.file.write("=" * 100 + "\n")

        self.file.close()
        self.csv_file.close()
        print(f"Log saved: {self.filepath} ({self.frame_count} frames)")


# ============================================================================
# Port Detection (imported from src.servo_controller)
# ============================================================================

auto_detect_port = _auto_detect_port


# ============================================================================
# Main Teleoperation Loop
# ============================================================================


def main():
    parser = argparse.ArgumentParser(description="Shadow Control - Real-time Teleoperation")
    parser.add_argument("--camera", "-c", type=int, default=1, help="Camera index (default: 1)")
    parser.add_argument("--port", "-p", help="Serial port (auto-detected if not specified)")
    parser.add_argument("--baudrate", "-b", type=int, default=500000, help="Servo baudrate")
    parser.add_argument("--smoothing", "-s", type=float, default=0.4, help="Smoothing factor 0-1 (default: 0.4)")
    parser.add_argument("--no-servo", action="store_true", help="Disable servo output (visualization only)")
    parser.add_argument("--log", "-l", action="store_true", default=True, help="Enable data logging to CSV")
    parser.add_argument("--debug", "-d", action="store_true", help="Enable debug logging")
    args = parser.parse_args()

    # Configure servo controller logging
    configure_logging(level=logging.DEBUG if args.debug else logging.INFO)

    # Download model if needed
    model_path = download_model()

    # Initialize pose detector (new Tasks API)
    base_options = mp_tasks.BaseOptions(model_asset_path=model_path)
    options = vision.PoseLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=0.90,
        min_tracking_confidence=0.90,
    )
    pose_landmarker = vision.PoseLandmarker.create_from_options(options)

    # Initialize servo controller
    servo_ctrl = None
    if not args.no_servo:
        port = args.port or auto_detect_port()
        if port:
            servo_ctrl = ServoController(port, args.baudrate)
            if servo_ctrl.connect():
                servo_ctrl.load_calibration(CALIBRATION_FILE)
                # Read current servo positions and use as calibration
                # This prevents the robot from jumping on startup
                logger.info("Reading current servo positions for calibration...")
                for servo_id in SERVO_MAPPING.keys():
                    pos = servo_ctrl.read_position_raw(servo_id)
                    config = SERVO_MAPPING[servo_id]
                    if pos is not None:
                        servo_ctrl.calibration[str(servo_id)]["zero_offset"] = pos
                        logger.info(f"  ID {servo_id} ({config['name']}): pos={pos} pose_key={config['pose_key']} scale={config['scale']}")
                    else:
                        logger.error(f"  ID {servo_id} ({config['name']}): FAILED TO READ")
                    time.sleep(0.01)
                logger.info("Servos calibrated from current positions.")
                # Initialize servo speed (CRITICAL: STS3215 needs non-zero speed to move)
                servo_ctrl.initialize_servos(list(SERVO_MAPPING.keys()) + [5, 7, 8, 10])
            else:
                servo_ctrl = None
        else:
            print("Warning: No serial port found - servo control disabled")

    # Initialize camera
    cap = cv2.VideoCapture(args.camera, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        print(f"Error: Cannot open camera {args.camera}")
        return 1

    print(f"Camera {args.camera} opened")

    # Initialize smoother
    smoother = AngleSmoother(alpha=args.smoothing)

    # Initialize data logger
    data_logger = DataLogger() if args.log else None

    # State
    servo_enabled = True
    last_time = time.time()
    fps = 0.0
    pose_zero_offsets = None  # Will be set from first valid frame

    print("\n" + "=" * 60)
    print("Shadow Control - Teleoperation Active")
    print("=" * 60)

    # Show servo mapping configuration
    print("\nSERVO MAPPING:")
    print("-" * 60)
    print(f"{'ID':<4} {'Name':<14} {'Pose Key':<12} {'Scale':<6}")
    print("-" * 60)
    for sid, cfg in SERVO_MAPPING.items():
        print(f"{sid:<4} {cfg['name']:<14} {cfg['pose_key']:<12} {cfg['scale']:+.1f}")
    print("-" * 60)

    print("\nControls:")
    print("  ESC   - Quit")
    print("  SPACE - Toggle servo control")
    print("  R     - Reset servos to neutral")
    print("  C     - Re-calibrate zero from current pose")
    if data_logger:
        print("\nAnnotation Keys (for logging):")
        print("  1 - arm_forward    2 - arm_side")
        print("  3 - elbow_bend     4 - both_arms")
        print("  5 - neutral        0 - other")
    print("=" * 60)
    print("\nStand in your NEUTRAL POSE for calibration...")
    print("=" * 60 + "\n")

    frame_timestamp_ms = 0
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Calculate FPS
            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - last_time, 0.001))
            last_time = now

            # Process pose with new Tasks API
            image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image_rgb)
            frame_timestamp_ms += 33  # ~30fps
            results = pose_landmarker.detect_for_video(mp_image, frame_timestamp_ms)

            pose_angles = {}
            servo_angles = {}
            encoder_angles = {}
            raw_positions = {}
            temperatures = {}

            if results.pose_landmarks and len(results.pose_landmarks) > 0:
                landmarks = results.pose_landmarks[0]  # First detected pose

                # Draw skeleton
                draw_landmarks(frame, landmarks)

                # Get pose angles
                raw_pose_angles = get_pose_angles(landmarks)
                pose_angles = raw_pose_angles.copy()

                # Debug: log raw pose angles every second
                if args.debug and frame_timestamp_ms % 1000 < 34:
                    logger.debug(f"RAW POSE: L_abd={raw_pose_angles.get('left_abd', 0):+6.1f} R_abd={raw_pose_angles.get('right_abd', 0):+6.1f} L_elb={raw_pose_angles.get('left_elbow', 0):+6.1f} R_elb={raw_pose_angles.get('right_elbow', 0):+6.1f}")

                # Warn about gimbal lock (NaN values)
                nan_keys = [k for k, v in raw_pose_angles.items() if np.isnan(v)]
                if nan_keys and args.debug:
                    logger.warning(f"GIMBAL LOCK: {', '.join(nan_keys)}")

                # Calibrate zero from first valid frame
                if pose_zero_offsets is None:
                    pose_zero_offsets = raw_pose_angles.copy()
                    logger.info("=" * 50)
                    logger.info("POSE CALIBRATION - Zero offsets captured:")
                    for key, val in pose_zero_offsets.items():
                        if not np.isnan(val):
                            logger.info(f"  {key}: {val:+.1f}°")
                    logger.info("=" * 50)

                # Subtract zero offsets (first frame becomes 0, 0, 0)
                # Negative values = below neutral, positive = above neutral
                for key in pose_angles:
                    if key in pose_zero_offsets and not np.isnan(pose_zero_offsets[key]):
                        pose_angles[key] = pose_angles[key] - pose_zero_offsets[key]

                # Debug: log pose after offset subtraction
                if args.debug and frame_timestamp_ms % 1000 < 34:
                    logger.debug(f"AFTER OFFSET: L_abd={pose_angles.get('left_abd', 0):+6.1f} R_abd={pose_angles.get('right_abd', 0):+6.1f} L_elb={pose_angles.get('left_elbow', 0):+6.1f} R_elb={pose_angles.get('right_elbow', 0):+6.1f}")

                # Smooth angles
                pose_angles = smoother.smooth_dict(pose_angles)

                # Debug: log pose after smoothing
                if args.debug and frame_timestamp_ms % 1000 < 34:
                    logger.debug(f"AFTER SMOOTH: L_abd={pose_angles.get('left_abd', 0):+6.1f} R_abd={pose_angles.get('right_abd', 0):+6.1f} L_elb={pose_angles.get('left_elbow', 0):+6.1f} R_elb={pose_angles.get('right_elbow', 0):+6.1f}")

                # Draw angle arcs on joints
                h, w = frame.shape[:2]
                
                # Helper to convert landmark to pixel coordinates
                def to_px(idx):
                    lm = landmarks[idx]
                    if lm.visibility > 0.5:
                        return (int(lm.x * w), int(lm.y * h))
                    return None
                
                # Left elbow flexion arc
                l_sh_px = to_px(11)
                l_el_px = to_px(13)
                l_wr_px = to_px(15)
                if l_sh_px and l_el_px and l_wr_px and not np.isnan(pose_angles.get("left_elbow", float("nan"))):
                    frame = draw_angle_arc(frame, l_sh_px, l_el_px, l_wr_px, 
                                          pose_angles.get("left_elbow", 0),
                                          color=(255, 0, 255), alpha=0.3, radius=30)
                
                # Right elbow flexion arc
                r_sh_px = to_px(12)
                r_el_px = to_px(14)
                r_wr_px = to_px(16)
                if r_sh_px and r_el_px and r_wr_px and not np.isnan(pose_angles.get("right_elbow", float("nan"))):
                    frame = draw_angle_arc(frame, r_sh_px, r_el_px, r_wr_px,
                                          pose_angles.get("right_elbow", 0),
                                          color=(255, 0, 255), alpha=0.3, radius=30)
                
                # Left shoulder abduction arc
                l_hip_px = to_px(23)
                if l_hip_px and l_sh_px and l_el_px and not np.isnan(pose_angles.get("left_abd", float("nan"))):
                    frame = draw_angle_arc(frame, l_hip_px, l_sh_px, l_el_px,
                                          pose_angles.get("left_abd", 0),
                                          color=(0, 255, 255), alpha=0.3, radius=35)
                
                # Right shoulder abduction arc
                r_hip_px = to_px(24)
                if r_hip_px and r_sh_px and r_el_px and not np.isnan(pose_angles.get("right_abd", float("nan"))):
                    frame = draw_angle_arc(frame, r_hip_px, r_sh_px, r_el_px,
                                          pose_angles.get("right_abd", 0),
                                          color=(0, 255, 255), alpha=0.3, radius=35)
                
                # Left shoulder extension arc (slightly offset to avoid overlap)
                if l_hip_px and l_sh_px and l_el_px and not np.isnan(pose_angles.get("left_ext", float("nan"))):
                    # Offset shoulder position slightly for extension arc
                    offset_sh_px = (l_sh_px[0] - 20, l_sh_px[1])
                    frame = draw_angle_arc(frame, l_hip_px, offset_sh_px, l_el_px,
                                          pose_angles.get("left_ext", 0),
                                          color=(255, 255, 0), alpha=0.3, radius=30)
                
                # Right shoulder extension arc (slightly offset to avoid overlap)
                if r_hip_px and r_sh_px and r_el_px and not np.isnan(pose_angles.get("right_ext", float("nan"))):
                    # Offset shoulder position slightly for extension arc
                    offset_sh_px = (r_sh_px[0] + 20, r_sh_px[1])
                    frame = draw_angle_arc(frame, r_hip_px, offset_sh_px, r_el_px,
                                          pose_angles.get("right_ext", 0),
                                          color=(255, 255, 0), alpha=0.3, radius=30)

                # Map to servo angles
                for servo_id, config in SERVO_MAPPING.items():
                    pose_key = config["pose_key"]
                    raw_angle = pose_angles.get(pose_key, 0)

                    if np.isnan(raw_angle):
                        raw_angle = 0

                    # Apply scale (offset is now handled by first-frame calibration)
                    servo_angle = config["scale"] * raw_angle
                    servo_angles[servo_id] = servo_angle

                # Debug: log servo commands every second
                if args.debug and frame_timestamp_ms % 1000 < 34:
                    cmd_str = " ".join([f"ID{sid}={ang:+6.1f}" for sid, ang in servo_angles.items()])
                    logger.debug(f"SERVO CMD: {cmd_str}")
                    # Specifically log elbow angles for debugging
                    logger.debug(f"ELBOW DEBUG: L_elbow_raw={raw_pose_angles.get('left_elbow', 0):+6.1f}° "
                               f"L_elbow_cal={pose_angles.get('left_elbow', 0):+6.1f}° "
                               f"L_servo={servo_angles.get(10, 0):+6.1f}° | "
                               f"R_elbow_raw={raw_pose_angles.get('right_elbow', 0):+6.1f}° "
                               f"R_elbow_cal={pose_angles.get('right_elbow', 0):+6.1f}° "
                               f"R_servo={servo_angles.get(7, 0):+6.1f}°")

                # Send to servos
                if servo_ctrl and servo_enabled and servo_angles:
                    servo_ctrl.write_angles(servo_angles)

                    # Debug: log rate limiting if it occurred
                    if args.debug and frame_timestamp_ms % 1000 < 34:
                        for sid, target in servo_angles.items():
                            actual = servo_ctrl.last_angles.get(sid)
                            if actual is not None and abs(actual - target) > 0.1:
                                logger.debug(f"RATE LIMITED: ID{sid} target={target:+.1f} actual={actual:+.1f}")

                # Read encoder positions (every 10th frame to reduce bus contention)
                encoder_angles = {}
                raw_positions = {}
                temperatures = {}
                if servo_ctrl and servo_ctrl.connected and frame_timestamp_ms % 333 < 34:  # ~every 10 frames
                    time.sleep(0.005)  # 5ms delay after writes before reading
                    encoder_angles = servo_ctrl.read_angles(SERVO_MAPPING.keys())
                    for sid in SERVO_MAPPING.keys():
                        raw_positions[sid] = servo_ctrl.read_position_raw(sid)

                    # Debug: log encoder feedback
                    if args.debug:
                        enc_str = " ".join([f"ID{sid}={ang:+6.1f}" for sid, ang in encoder_angles.items() if ang is not None])
                        logger.debug(f"ENCODER: {enc_str}")

                # Read temperatures less frequently (every ~3 seconds)
                if servo_ctrl and servo_ctrl.connected and frame_timestamp_ms % 3000 < 34:
                    for sid in SERVO_MAPPING.keys():
                        temp = servo_ctrl.read_temperature(sid)
                        if temp is not None:
                            temperatures[sid] = temp

                    # Check servo health and log warnings
                    for sid in SERVO_MAPPING.keys():
                        warnings = servo_ctrl.check_servo_health(sid)
                        for warning in warnings:
                            logger.warning(warning)

                # Log data and check for frame capture
                if data_logger:
                    data_logger.log(pose_angles, servo_angles, encoder_angles)
                    data_logger.check_capture(frame, pose_angles, servo_angles)

            # Draw HUD with encoder readings and temperatures
            draw_hud(frame, pose_angles, servo_angles, servo_enabled and servo_ctrl is not None, fps, encoder_angles, raw_positions, temperatures)

            # Show frame
            cv2.imshow("Shadow Control - Teleoperation", frame)

            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == ord(" "):  # SPACE - toggle servo
                servo_enabled = not servo_enabled
                logger.info(f"Servo control: {'ENABLED' if servo_enabled else 'DISABLED'}")
            elif key == ord("r") or key == ord("R"):  # R - reset to neutral positions
                if servo_ctrl:
                    print("Resetting servos to neutral positions...")
                    # Reset ALL servos with neutral positions (including disabled ones)
                    for sid, target in NEUTRAL_POSITIONS.items():
                        servo_ctrl.write_position_raw(sid, target)
                        print(f"  ID {sid} -> {target}")
                        time.sleep(0.05)
                    # Also update calibration to match neutral
                    for sid, target in NEUTRAL_POSITIONS.items():
                        if str(sid) in servo_ctrl.calibration:
                            servo_ctrl.calibration[str(sid)]["zero_offset"] = target
                    print("Calibration updated to neutral positions.")
            elif key == ord("c") or key == ord("C"):  # C - recalibrate
                pose_zero_offsets = None
                smoother.values = {}  # Reset smoother too
                print("Re-calibrating... stand in neutral pose.")
            elif data_logger and key in ANNOTATION_LABELS:
                data_logger.set_annotation(ANNOTATION_LABELS[key])

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        pose_landmarker.close()
        if data_logger:
            data_logger.close()
        if servo_ctrl:
            # Return to neutral before disconnecting
            print("Returning servos to neutral...")
            servo_ctrl.write_angles({sid: 0 for sid in SERVO_MAPPING.keys()}, force=True)
            time.sleep(0.5)
            servo_ctrl.disconnect()

    print("Teleoperation ended")
    return 0


if __name__ == "__main__":
    sys.exit(main())
