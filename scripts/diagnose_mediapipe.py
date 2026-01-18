#!/usr/bin/env python3
"""
MediaPipe Pose Diagnostic Tool
===============================
Interactive script to log MediaPipe pose angles on keypress.
Uses the SAME pose detection and angle calculations as teleoperate.py.

Usage:
  python scripts/diagnose_mediapipe.py
  python scripts/diagnose_mediapipe.py --camera 0

Controls:
  SPACE - Log current angles with custom annotation
  n     - Log as "neutral" (arms at sides)
  a     - Log as "abducted" (arms out to sides)
  e     - Log as "extended" (arms forward)
  b     - Log as "back" (arms behind)
  u     - Log as "up" (arms raised)
  d     - Log as "down" (arms lowered)
  q     - Quit

Output saved to: logs/mediapipe_poses_<timestamp>.txt
"""

import argparse
import os
import sys
import cv2
import numpy as np
import urllib.request
from datetime import datetime

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import mediapipe as mp
from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision

# Model configuration - use heavy model like teleoperate.py
MODEL_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "models")
MODEL_FILE = os.path.join(MODEL_DIR, "pose_landmarker_heavy.task")
MODEL_URL = "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task"

# Pose connections for drawing skeleton (same as teleoperate.py)
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
    try:
        urllib.request.urlretrieve(MODEL_URL, MODEL_FILE)
        print(f"Model saved to {MODEL_FILE}")
    except Exception as e:
        print(f"ERROR downloading model: {e}")
        return None
    return MODEL_FILE


# =============================================================================
# Angle calculations - COPIED FROM teleoperate.py for consistency
# =============================================================================

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
    """Calculate elbow flexion angle. 0° = straight, increases as elbow bends."""
    raw_angle = angle_between(shoulder - elbow, wrist - elbow)
    return 180.0 - raw_angle if not np.isnan(raw_angle) else raw_angle


def shoulder_abduction(hip, shoulder, elbow):
    """
    Calculate shoulder abduction (horizontal plane movement).
    Returns 0-180°, or NaN if gimbal locked.
    0° = arm at side, 90° = horizontal out, 180° = across body
    """
    arm = elbow - shoulder
    arm_full = arm.copy()
    arm = arm.copy()

    arm_full_norm = np.linalg.norm(arm_full)
    if arm_full_norm < 1e-6:
        return float("nan")

    # Project to XY plane (remove Z/depth)
    arm[2] = 0
    arm_norm = np.linalg.norm(arm)

    # Gimbal lock check
    projection_ratio = arm_norm / arm_full_norm
    if projection_ratio < 0.3:
        return float("nan")

    # Reference: downward (positive Y in MediaPipe)
    down_ref = np.array([0.0, 1.0, 0.0])
    arm_xy = arm / arm_norm
    dot = np.clip(np.dot(down_ref, arm_xy), -1.0, 1.0)
    angle = np.degrees(np.arccos(dot))

    return min(180.0, max(0.0, angle))


def shoulder_extension(hip, shoulder, elbow):
    """
    Calculate shoulder extension (sagittal/YZ plane).
    Returns 0-180°, or NaN if gimbal locked.
    0° = arm down, 90° = horizontal forward, 180° = overhead
    """
    arm = elbow - shoulder
    arm_full = arm.copy()
    arm = arm.copy()

    arm_full_norm = np.linalg.norm(arm_full)
    if arm_full_norm < 1e-6:
        return float("nan")

    # Project to YZ plane (remove X/side)
    arm[0] = 0
    arm_norm = np.linalg.norm(arm)

    # Gimbal lock check
    projection_ratio = arm_norm / arm_full_norm
    if projection_ratio < 0.3:
        return float("nan")

    # Reference: downward
    down_ref = np.array([0.0, 1.0, 0.0])
    arm_yz = arm / arm_norm
    dot = np.clip(np.dot(down_ref, arm_yz), -1.0, 1.0)
    angle = np.degrees(np.arccos(dot))

    return min(180.0, max(0.0, angle))


def get_pose_angles(landmarks):
    """Extract joint angles from MediaPipe pose landmarks."""
    def p(i):
        lm = landmarks[i]
        return np.array([lm.x, lm.y, lm.z], dtype=np.float32)

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


# =============================================================================
# Visualization - matches teleoperate.py
# =============================================================================

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


def draw_angle_arc(frame, p1_px, p2_px, p3_px, angle_deg, color=(255, 0, 255), alpha=0.4):
    """Draw an angle arc between three points (p2 is the vertex)."""
    if np.isnan(angle_deg):
        return frame

    x1, y1 = p1_px
    x2, y2 = p2_px
    x3, y3 = p3_px

    a = np.array([x1 - x2, y1 - y2], dtype=float)
    b = np.array([x3 - x2, y3 - y2], dtype=float)

    if np.linalg.norm(a) < 1e-6 or np.linalg.norm(b) < 1e-6:
        return frame

    base_radius = int(min(max(np.linalg.norm(a) * 0.15, 15), 50))
    a_unit = a / np.linalg.norm(a)
    b_unit = b / np.linalg.norm(b)

    arc_points = []
    for i in range(31):
        t = i / 30
        vec = (1 - t) * a_unit + t * b_unit
        if np.linalg.norm(vec) < 1e-6:
            continue
        vec = vec / np.linalg.norm(vec)
        px = int(x2 + vec[0] * base_radius)
        py = int(y2 + vec[1] * base_radius)
        arc_points.append((px, py))

    if len(arc_points) < 2:
        return frame

    # Draw filled arc
    poly = [(x2, y2)] + arc_points + [(x2, y2)]
    overlay = frame.copy()
    cv2.fillPoly(overlay, [np.array(poly, dtype=np.int32)], color)
    frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)
    cv2.polylines(frame, [np.array(arc_points, dtype=np.int32)], False, color, 2)

    # Draw angle text
    text = f"{int(angle_deg)}"
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, text, (x2 - 15, y2 - base_radius - 5), font, 0.5, color, 2)

    return frame


def put_text_bg(img, text, org, font_scale=0.7, color=(255, 255, 255)):
    """Draw text with background."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv2.getTextSize(text, font, font_scale, 2)
    x, y = org
    cv2.rectangle(img, (x - 4, y - th - 4), (x + tw + 4, y + 4), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), font, font_scale, color, 2, cv2.LINE_AA)


def draw_hud(frame, angles, fps):
    """Draw heads-up display with angles."""
    h, w = frame.shape[:2]

    # Title
    put_text_bg(frame, f"MEDIAPIPE DIAGNOSTIC  |  FPS: {fps:.1f}", (15, 30), 0.7, (0, 255, 255))

    # Left side angles
    y = 70
    for key in ["left_ext", "left_abd", "left_elbow"]:
        val = angles.get(key, float("nan"))
        label = key.replace("left_", "L_")
        if not np.isnan(val):
            text = f"{label}: {val:5.1f}"
            color = (0, 255, 255)
        else:
            text = f"{label}:   NaN"
            color = (0, 0, 255)
        put_text_bg(frame, text, (15, y), 0.6, color)
        y += 30

    # Right side angles
    y = 70
    for key in ["right_ext", "right_abd", "right_elbow"]:
        val = angles.get(key, float("nan"))
        label = key.replace("right_", "R_")
        if not np.isnan(val):
            text = f"{label}: {val:5.1f}"
            color = (0, 255, 255)
        else:
            text = f"{label}:   NaN"
            color = (0, 0, 255)
        put_text_bg(frame, text, (w - 150, y), 0.6, color)
        y += 30

    # Controls hint
    put_text_bg(frame, "n=neutral a=abduct e=extend b=back u=up d=down q=quit", (15, h - 15), 0.5)


def main():
    parser = argparse.ArgumentParser(description="MediaPipe Pose Diagnostic Tool")
    parser.add_argument("--camera", "-c", type=int, default=1, help="Camera index (default: 1)")
    args = parser.parse_args()

    # Download model
    model_path = download_model()
    if not model_path:
        return

    # Initialize pose detector - SAME settings as teleoperate.py
    base_options = mp_tasks.BaseOptions(model_asset_path=model_path)
    options = vision.PoseLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.VIDEO,  # VIDEO mode, not LIVE_STREAM
        num_poses=1,
        min_pose_detection_confidence=0.90,  # High confidence like teleoperate
        min_tracking_confidence=0.90,
    )
    pose_landmarker = vision.PoseLandmarker.create_from_options(options)

    # Initialize camera
    cap = cv2.VideoCapture(args.camera, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        print(f"ERROR: Cannot open camera {args.camera}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    print(f"Camera {args.camera} opened")

    # Create log file
    log_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "logs")
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(log_dir, f"mediapipe_poses_{timestamp}.txt")

    print("\n" + "=" * 80)
    print("  MEDIAPIPE POSE DIAGNOSTIC TOOL")
    print("  (Using same detection as teleoperate.py)")
    print("=" * 80)
    print(f"Output: {log_file}")
    print("=" * 80 + "\n")

    # Annotation shortcuts
    annotations = {
        ord('n'): "neutral",
        ord('a'): "abducted",
        ord('e'): "extended",
        ord('b'): "back",
        ord('u'): "up",
        ord('d'): "down",
    }

    import time
    frame_timestamp_ms = 0
    last_time = time.time()
    fps = 0.0

    with open(log_file, 'w') as f:
        f.write("=" * 100 + "\n")
        f.write(f"  MediaPipe Pose Log - {timestamp}\n")
        f.write("=" * 100 + "\n\n")
        f.write("Angles: 0° = arm down, 90° = horizontal, 180° = overhead\n")
        f.write("-" * 100 + "\n")
        f.write("Entry │  L_ext  │  L_abd  │ L_elbow │  R_ext  │  R_abd  │ R_elbow │ Annotation\n")
        f.write("-" * 100 + "\n")

        entry_num = 0
        print("Ready. Press keys to log poses...")

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # FPS calculation
            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - last_time, 0.001))
            last_time = now

            # Process with MediaPipe (same as teleoperate.py)
            image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image_rgb)
            frame_timestamp_ms += 33
            results = pose_landmarker.detect_for_video(mp_image, frame_timestamp_ms)

            angles = {}

            if results.pose_landmarks and len(results.pose_landmarks) > 0:
                landmarks = results.pose_landmarks[0]
                world_landmarks = results.pose_world_landmarks[0]

                # Draw skeleton
                draw_landmarks(frame, landmarks)

                # Get angles from world landmarks
                angles = get_pose_angles(world_landmarks)

                # Draw angle arcs
                h, w = frame.shape[:2]

                def to_px(idx):
                    lm = landmarks[idx]
                    if lm.visibility > 0.5:
                        return (int(lm.x * w), int(lm.y * h))
                    return None

                # Left elbow arc
                l_sh, l_el, l_wr = to_px(11), to_px(13), to_px(15)
                if l_sh and l_el and l_wr:
                    frame = draw_angle_arc(frame, l_sh, l_el, l_wr,
                                           angles.get("left_elbow", float("nan")),
                                           color=(255, 0, 255))

                # Right elbow arc
                r_sh, r_el, r_wr = to_px(12), to_px(14), to_px(16)
                if r_sh and r_el and r_wr:
                    frame = draw_angle_arc(frame, r_sh, r_el, r_wr,
                                           angles.get("right_elbow", float("nan")),
                                           color=(255, 0, 255))

                # Left shoulder abduction arc
                l_hip = to_px(23)
                if l_hip and l_sh and l_el:
                    frame = draw_angle_arc(frame, l_hip, l_sh, l_el,
                                           angles.get("left_abd", float("nan")),
                                           color=(0, 255, 255))

                # Right shoulder abduction arc
                r_hip = to_px(24)
                if r_hip and r_sh and r_el:
                    frame = draw_angle_arc(frame, r_hip, r_sh, r_el,
                                           angles.get("right_abd", float("nan")),
                                           color=(0, 255, 255))

            # Draw HUD
            draw_hud(frame, angles, fps)

            cv2.imshow("MediaPipe Pose Diagnostic", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            if key in annotations and angles:
                entry_num += 1
                annotation = annotations[key]

                def fmt(val):
                    if not np.isnan(val):
                        return f"{val:7.1f}"
                    return "    NaN"

                line = (
                    f"{entry_num:5d} │ "
                    f"{fmt(angles.get('left_ext', float('nan')))} │ "
                    f"{fmt(angles.get('left_abd', float('nan')))} │ "
                    f"{fmt(angles.get('left_elbow', float('nan')))} │ "
                    f"{fmt(angles.get('right_ext', float('nan')))} │ "
                    f"{fmt(angles.get('right_abd', float('nan')))} │ "
                    f"{fmt(angles.get('right_elbow', float('nan')))} │ "
                    f"{annotation}"
                )
                print(line)
                f.write(line + "\n")
                f.flush()

    cap.release()
    cv2.destroyAllWindows()
    pose_landmarker.close()
    print(f"\nLog saved to: {log_file}")


if __name__ == "__main__":
    main()
