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
import json
import os
import sys
import time
import urllib.request

import cv2
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
#   R Shoulder Extension -> Servo 5 (Right Shoulder)
#   R Shoulder Abduction -> Servo 6 (Right Elbow)
#   R Elbow Flexion      -> Servo 7 (Right Wrist)
#   L Shoulder Extension -> Servo 8 (Left Shoulder)
#   L Shoulder Abduction -> Servo 9 (Left Elbow)
#   L Elbow Flexion      -> Servo 10 (Left Wrist)

SERVO_MAPPING = {
    # Offsets are calibrated from first frame automatically (initial angles become 0)
    # Scale: positive = same direction, negative = inverted
    # Extension: 0° to 180° range (at side to forward/up)
    # Abduction: 0° to 180° range (down at side to straight up)
    # Elbow flexion: 0° to 180° range (straight to fully bent)
    5: {"name": "R_Shoulder", "pose_key": "right_ext", "scale": 1.0, "min": 0, "max": 180},
    6: {"name": "R_Elbow", "pose_key": "right_abd", "scale": 1.0, "min": 0, "max": 180},
    7: {"name": "R_Wrist", "pose_key": "right_elbow", "scale": 1.0, "min": 0, "max": 180},
    8: {"name": "L_Shoulder", "pose_key": "left_ext", "scale": 1.0, "min": 0, "max": 180},
    9: {"name": "L_Elbow", "pose_key": "left_abd", "scale": 1.0, "min": 0, "max": 180},
    10: {"name": "L_Wrist", "pose_key": "left_elbow", "scale": 1.0, "min": 0, "max": 180},
}

CALIBRATION_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "servo_calibration.json",
)

# Model file path and URL
MODEL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models")
MODEL_FILE = os.path.join(MODEL_DIR, "pose_landmarker_lite.task")
MODEL_URL = "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task"

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
    """Calculate elbow flexion angle."""
    return angle_between(shoulder - elbow, wrist - elbow)


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
    Returns angle in range 0° to 180°:
    - 0° = arm at side (aligned with torso vertically, no horizontal deviation)
    - 90° = arm horizontal out to side (perpendicular to torso)
    - 180° = arm across body (maximum horizontal adduction)
    
    Coordinate System (XY/horizontal plane):
    - Measures angle from vertical reference (downward) to arm direction in horizontal plane
    - Based on "Horizontal Adduction" diagram where 0° = arms at side, 90° = horizontal, 180° = across body
    """
    arm = elbow - shoulder
    arm = arm.copy()
    
    # Project to XY plane (horizontal plane, removes depth/z component)
    arm[2] = 0
    
    # Check if arm vector is valid
    arm_norm = np.linalg.norm(arm)
    if arm_norm < 1e-6:
        return float("nan")
    
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
    Returns angle in range 0° to 180°:
    - 0° = arm down at side (aligned with torso vertically)
    - 90° = arm horizontal forward (perpendicular to torso)
    - 180° = arm straight up overhead (opposite to torso direction)
    
    Coordinate System (YZ/sagittal plane):
    - Measures angle from vertical reference (downward) to arm direction in sagittal plane
    - Based on "Vertical Elevation" diagram:
      * 0° = arm down (aligned with vertical)
      * 90° = arm horizontal forward (perpendicular to vertical)
      * 180° = arm up (opposite to vertical)
    """
    arm = elbow - shoulder   # Vector from shoulder to elbow
    arm = arm.copy()
    
    # Project to YZ plane (sagittal plane, removes side/x component)
    arm[0] = 0
    
    # Check if projected arm vector is valid
    arm_norm = np.linalg.norm(arm)
    if arm_norm < 1e-6:  # Arm is perpendicular to YZ plane (horizontal out to side)
        return 0.0  # No vertical elevation component
    
    # Reference direction: downward in YZ plane (positive Y direction in MediaPipe)
    # In MediaPipe, Y increases downward, so [0, 1, 0] is "down"
    down_ref = np.array([0.0, 1.0, 0.0])
    
    # Normalize arm vector in YZ plane
    arm_yz = arm / arm_norm
    
    # Calculate angle from vertical (down) to arm direction in YZ plane
    # This gives us the extension angle: 0° when arm is down, 90° when horizontal forward, 180° when up
    dot = np.clip(np.dot(down_ref, arm_yz), -1.0, 1.0)
    angle = np.degrees(np.arccos(dot))
    
    # angle_between returns 0-180°, which is what we want
    # 0° = arm pointing down (at side)
    # 90° = arm pointing forward (horizontal)
    # 180° = arm pointing up (overhead)
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
# Servo Control
# ============================================================================

try:
    from scservo_sdk import PortHandler, PacketHandler

    HAS_SDK = True
except ImportError:
    try:
        from feetech_servo_sdk import PortHandler, PacketHandler

        HAS_SDK = True
    except ImportError:
        HAS_SDK = False


class ServoController:
    """Servo controller for teleoperation."""

    ADDR_GOAL_POSITION = 42
    ADDR_CURRENT_POSITION = 56

    def __init__(self, port, baudrate=500000):
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.packet_handler = None
        self.calibration = {}
        self.connected = False

    def connect(self):
        """Open serial connection to servos."""
        if not HAS_SDK:
            print("Warning: scservo_sdk not found - servo control disabled")
            return False

        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(1)

        if not self.port_handler.openPort():
            print(f"Error: Failed to open port {self.port}")
            return False

        self.port_handler.setBaudRate(self.baudrate)
        self.connected = True
        print(f"Connected to servos on {self.port} at {self.baudrate} bps")
        return True

    def disconnect(self):
        """Close serial connection."""
        if self.port_handler:
            self.port_handler.closePort()
        self.connected = False

    def load_calibration(self, filepath):
        """Load calibration file with zero offsets."""
        if os.path.exists(filepath):
            with open(filepath, "r") as f:
                self.calibration = json.load(f)
            print(f"Loaded calibration: {list(self.calibration.keys())}")
            return True
        print(f"Warning: Calibration file not found: {filepath}")
        return False

    def angle_to_steps(self, angle):
        """Convert angle (degrees) to servo steps."""
        return int(angle * STEPS_PER_REV / 360)

    def write_angle(self, servo_id, angle):
        """Write angle to servo (relative to calibrated zero)."""
        if not self.connected:
            return False

        # Don't actuate if angle is less than 10 degrees (dead zone)
        if abs(angle) < 10.0:
            return False

        key = str(servo_id)
        if key not in self.calibration:
            return False

        zero_offset = self.calibration[key]["zero_offset"]
        target_steps = zero_offset + self.angle_to_steps(angle)

        # Clamp to valid range
        target_steps = max(0, min(65535, target_steps))

        self.packet_handler.write2ByteTxRx(
            self.port_handler, servo_id, self.ADDR_GOAL_POSITION, int(target_steps)
        )
        return True

    def write_angles(self, angle_dict):
        """Write multiple angles at once. angle_dict: {servo_id: angle}"""
        for servo_id, angle in angle_dict.items():
            self.write_angle(servo_id, angle)


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


def put_text_with_bg(img, text, org, font_scale=0.55, thickness=1, pad=4, color=(255, 255, 255)):
    """Draw text with background for readability."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = org
    cv2.rectangle(img, (x - pad, y - th - pad), (x + tw + pad, y + baseline + pad), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)


def draw_hud(frame, pose_angles, servo_angles, servo_enabled, fps):
    """Draw heads-up display with angle info."""
    h, w = frame.shape[:2]

    # Status bar
    status_color = (0, 255, 0) if servo_enabled else (0, 0, 255)
    status_text = "SERVOS: ON" if servo_enabled else "SERVOS: OFF"
    put_text_with_bg(frame, f"{status_text}  |  FPS: {fps:.1f}", (12, 28), color=status_color)

    # Pose angles (left column)
    y = 60
    put_text_with_bg(frame, "Pose Angles", (12, y), font_scale=0.5)
    y += 22
    for key in ["left_ext", "left_abd", "left_elbow", "right_ext", "right_abd", "right_elbow"]:
        val = pose_angles.get(key, float("nan"))
        text = f"{key}: {val:6.1f}" if not np.isnan(val) else f"{key}:    N/A"
        put_text_with_bg(frame, text, (12, y), font_scale=0.45)
        y += 18

    # Servo commands (right column)
    y = 60
    put_text_with_bg(frame, "Servo Commands", (w - 180, y), font_scale=0.5)
    y += 22
    for servo_id, config in SERVO_MAPPING.items():
        angle = servo_angles.get(servo_id, 0)
        text = f"ID{servo_id} {config['name']}: {angle:6.1f}"
        put_text_with_bg(frame, text, (w - 180, y), font_scale=0.45)
        y += 18

    # Controls hint
    put_text_with_bg(frame, "ESC:quit  SPACE:toggle  R:reset", (12, h - 12), font_scale=0.45)


# ============================================================================
# Port Detection
# ============================================================================


def auto_detect_port():
    """Auto-detect serial port for servo adapter."""
    import glob

    # macOS USB modem
    ports = glob.glob("/dev/tty.usbmodem*")
    if ports:
        return ports[0]

    # Linux USB
    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    if ports:
        return ports[0]

    # Milk-V UART
    if os.path.exists("/dev/ttyS2"):
        return "/dev/ttyS2"

    return None


# ============================================================================
# Main Teleoperation Loop
# ============================================================================


def main():
    parser = argparse.ArgumentParser(description="Shadow Control - Real-time Teleoperation")
    parser.add_argument("--camera", "-c", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--port", "-p", help="Serial port (auto-detected if not specified)")
    parser.add_argument("--baudrate", "-b", type=int, default=500000, help="Servo baudrate")
    parser.add_argument("--smoothing", "-s", type=float, default=0.4, help="Smoothing factor 0-1 (default: 0.4)")
    parser.add_argument("--no-servo", action="store_true", help="Disable servo output (visualization only)")
    args = parser.parse_args()

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

    # State
    servo_enabled = True
    last_time = time.time()
    fps = 0.0
    pose_zero_offsets = None  # Will be set from first valid frame

    print("\n" + "=" * 50)
    print("Shadow Control - Teleoperation Active")
    print("=" * 50)
    print("Controls:")
    print("  ESC   - Quit")
    print("  SPACE - Toggle servo control")
    print("  R     - Reset servos to neutral")
    print("  C     - Re-calibrate zero from current pose")
    print("=" * 50)
    print("\nStand in your NEUTRAL POSE for calibration...")
    print("=" * 50 + "\n")

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

            if results.pose_landmarks and len(results.pose_landmarks) > 0:
                landmarks = results.pose_landmarks[0]  # First detected pose

                # Draw skeleton
                draw_landmarks(frame, landmarks)

                # Get pose angles
                pose_angles = get_pose_angles(landmarks)

                # Calibrate zero from first valid frame
                if pose_zero_offsets is None:
                    pose_zero_offsets = pose_angles.copy()
                    print("CALIBRATED! Zero offsets captured from current pose.")
                    print(f"  Offsets: {pose_zero_offsets}")

                # Subtract zero offsets (first frame becomes 0, 0, 0)
                for key in pose_angles:
                    if key in pose_zero_offsets and not np.isnan(pose_zero_offsets[key]):
                        pose_angles[key] = pose_angles[key] - pose_zero_offsets[key]
                        # Clamp to 0-180 range to prevent negative values
                        if not np.isnan(pose_angles[key]):
                            pose_angles[key] = max(0.0, min(180.0, pose_angles[key]))

                # Smooth angles
                pose_angles = smoother.smooth_dict(pose_angles)

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

                    # Clamp to safe range
                    servo_angle = max(config["min"], min(config["max"], servo_angle))

                    servo_angles[servo_id] = servo_angle

                # Send to servos
                if servo_ctrl and servo_enabled and servo_angles:
                    servo_ctrl.write_angles(servo_angles)

            # Draw HUD
            draw_hud(frame, pose_angles, servo_angles, servo_enabled and servo_ctrl is not None, fps)

            # Show frame
            cv2.imshow("Shadow Control - Teleoperation", frame)

            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == ord(" "):  # SPACE - toggle servo
                servo_enabled = not servo_enabled
                print(f"Servo control: {'ENABLED' if servo_enabled else 'DISABLED'}")
            elif key == ord("r") or key == ord("R"):  # R - reset
                if servo_ctrl:
                    print("Resetting servos to neutral...")
                    servo_ctrl.write_angles({sid: 0 for sid in SERVO_MAPPING.keys()})
            elif key == ord("c") or key == ord("C"):  # C - recalibrate
                pose_zero_offsets = None
                smoother.values = {}  # Reset smoother too
                print("Re-calibrating... stand in neutral pose.")

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        pose_landmarker.close()
        if servo_ctrl:
            # Return to neutral before disconnecting
            print("Returning servos to neutral...")
            servo_ctrl.write_angles({sid: 0 for sid in SERVO_MAPPING.keys()})
            time.sleep(0.5)
            servo_ctrl.disconnect()

    print("Teleoperation ended")
    return 0


if __name__ == "__main__":
    sys.exit(main())
