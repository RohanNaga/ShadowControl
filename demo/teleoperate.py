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
    # Offsets are calibrated from first frame automatically
    # Scale: positive = same direction, negative = inverted
    5: {"name": "R_Shoulder", "pose_key": "right_ext", "scale": 1.0, "min": -90, "max": 90},
    6: {"name": "R_Elbow", "pose_key": "right_abd", "scale": 1.0, "min": -90, "max": 90},
    7: {"name": "R_Wrist", "pose_key": "right_elbow", "scale": 1.0, "min": -90, "max": 90},
    8: {"name": "L_Shoulder", "pose_key": "left_ext", "scale": 1.0, "min": -90, "max": 90},
    9: {"name": "L_Elbow", "pose_key": "left_abd", "scale": 1.0, "min": -90, "max": 90},
    10: {"name": "L_Wrist", "pose_key": "left_elbow", "scale": 1.0, "min": -90, "max": 90},
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


def shoulder_abduction(hip, shoulder, elbow):
    """Calculate shoulder abduction (side-to-side in XY plane)."""
    torso = shoulder - hip
    arm = elbow - shoulder
    torso = torso.copy()
    arm = arm.copy()
    torso[2] = 0
    arm[2] = 0
    return angle_between(torso, arm)


def shoulder_extension(hip, shoulder, elbow):
    """Calculate shoulder extension (front-to-back in YZ plane)."""
    torso = shoulder - hip
    arm = elbow - shoulder
    torso = torso.copy()
    arm = arm.copy()
    torso[0] = 0
    arm[0] = 0
    return angle_between(torso, arm)


def get_pose_angles(landmarks):
    """
    Extract joint angles from MediaPipe pose landmarks.

    Args:
        landmarks: List of NormalizedLandmark from PoseLandmarker result

    Returns dict with keys:
        left_abd, right_abd - shoulder abduction
        left_ext, right_ext - shoulder extension
        left_elbow, right_elbow - elbow flexion
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
    }


def draw_landmarks(frame, landmarks):
    """Draw pose landmarks and connections on frame."""
    h, w = frame.shape[:2]

    # Draw connections
    for start_idx, end_idx in POSE_CONNECTIONS:
        if start_idx < len(landmarks) and end_idx < len(landmarks):
            start = landmarks[start_idx]
            end = landmarks[end_idx]
            start_point = (int(start.x * w), int(start.y * h))
            end_point = (int(end.x * w), int(end.y * h))
            cv2.line(frame, start_point, end_point, (0, 255, 0), 2)

    # Draw landmarks
    for i, lm in enumerate(landmarks):
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
    parser.add_argument("--camera", "-c", type=int, default=1, help="Camera index (default: 1)")
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
        min_pose_detection_confidence=0.5,
        min_tracking_confidence=0.5,
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
    cap = cv2.VideoCapture(args.camera)
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

                # Smooth angles
                pose_angles = smoother.smooth_dict(pose_angles)

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
