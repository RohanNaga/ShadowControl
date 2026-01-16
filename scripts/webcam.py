from ultralytics import YOLO
import cv2
import torch
import numpy as np
import os

# Fix temp directory issue
os.environ['TMPDIR'] = '/tmp'

# Try to load depth model using torch.hub (simpler, no transformers needed)
depth_model = None
midas_transform = None
DEPTH_AVAILABLE = False

try:
    print("\n" + "="*60)
    print("Loading depth estimation model (MiDaS via torch.hub)...")
    print("="*60)
    
    # Fix SSL certificate issues for macOS
    import ssl
    import certifi
    import os
    
    # Set environment variables for SSL certificates
    cert_path = certifi.where()
    os.environ['SSL_CERT_FILE'] = cert_path
    os.environ['REQUESTS_CA_BUNDLE'] = cert_path
    
    # Also try to patch urllib for torch.hub
    try:
        import urllib.request
        ssl_context = ssl.create_default_context(cafile=cert_path)
        # This might help with torch.hub's internal requests
        original_urlopen = urllib.request.urlopen
        def urlopen_with_ssl(*args, **kwargs):
            if 'context' not in kwargs:
                kwargs['context'] = ssl_context
            return original_urlopen(*args, **kwargs)
        urllib.request.urlopen = urlopen_with_ssl
    except:
        pass
    
    midas_model_type = "MiDaS_small"  # Smaller model (~70MB vs 1.28GB for DPT_Large)
    
    print("Downloading model (this may take a minute on first run)...")
    print("Note: If SSL error occurs, depth estimation will be disabled")
    
    depth_model = torch.hub.load("intel-isl/MiDaS", midas_model_type, trust_repo=True)
    depth_model.eval()
    print("✓ Model downloaded")
    
    # Load transforms
    print("Loading transforms...")
    midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
    if midas_model_type == "DPT_Large" or midas_model_type == "DPT_Hybrid":
        midas_transform = midas_transforms.dpt_transform
    else:
        midas_transform = midas_transforms.small_transform
    print("✓ Transforms loaded")
    
    if torch.backends.mps.is_available():
        depth_model = depth_model.to("mps")
        print("✓ Model moved to MPS")
    
    print("="*60)
    print("✓ MiDaS depth model loaded successfully!")
    print("="*60 + "\n")
    DEPTH_AVAILABLE = True
except Exception as e:
    print("\n" + "="*60)
    print(f"⚠ Depth model not available: {type(e).__name__}")
    error_msg = str(e)
    if len(error_msg) > 150:
        error_msg = error_msg[:150] + "..."
    print(f"   Error: {error_msg}")
    if "SSL" in str(e) or "certificate" in str(e).lower():
        print("\n   This is due to SSL certificate verification issues.")
        print("   Possible solutions:")
        print("   1. Install/update certificates: /Applications/Python\\ 3.13/Install\\ Certificates.command")
        print("   2. Or use a pre-downloaded model")
    elif "No space left" in str(e) or "OSError" in str(type(e).__name__):
        print("\n   This is due to insufficient disk space.")
        print("   The model requires ~70MB (MiDaS_small) or ~1.3GB (DPT_Large).")
        print("   Possible solutions:")
        print("   1. Free up disk space")
        print("   2. Clear PyTorch cache: rm -rf ~/.cache/torch/hub")
        print("   3. Use a smaller model (already using MiDaS_small)")
    print("   Depth estimation will be disabled.")
    print("   Continuing with YOLOv8-pose only (2D keypoints)")
    print("="*60 + "\n")
    DEPTH_AVAILABLE = False

# COCO keypoint names (17 keypoints)
KEYPOINT_NAMES = [
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle"
]

# Load YOLOv8n-pose model
model = YOLO("yolov8n-pose.pt")

# Optional: move model to Apple Silicon (MPS) if available
if torch.backends.mps.is_available():
    device = "mps"
else:
    device = "cpu"

print(f"Using device: {device}")
print(f"\n{'='*60}")
print(f"DEPTH MODEL STATUS: {'✓ AVAILABLE' if DEPTH_AVAILABLE else '✗ NOT AVAILABLE'}")
print(f"{'='*60}\n")

# Try to open webcam (macOS: AVFoundation backend)
cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)

if not cap.isOpened():
    print("Error: Could not open webcam. Check macOS camera permissions for your terminal/IDE.")
    exit(1)

print("Starting webcam feed... press 'q' to quit.")

def calculate_angle(p1, p2, p3):
    """Calculate angle at p2 formed by p1-p2-p3 in degrees"""
    if p1[2] < 0.5 or p2[2] < 0.5 or p3[2] < 0.5:
        return None  # Keypoint not visible
    
    v1 = np.array([p1[0], p1[1]]) - np.array([p2[0], p2[1]])
    v2 = np.array([p3[0], p3[1]]) - np.array([p2[0], p2[1]])
    
    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
    angle = np.arccos(np.clip(cos_angle, -1, 1))
    return np.degrees(angle)

def calculate_distance(p1, p2):
    """Calculate Euclidean distance between two keypoints"""
    return np.linalg.norm(np.array([p1[0], p1[1]]) - np.array([p2[0], p2[1]]))

def estimate_depth_map(frame):
    """Estimate depth map using MiDaS model"""
    if not DEPTH_AVAILABLE or depth_model is None:
        return None
    
    try:
        # Convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Apply MiDaS transform
        if midas_transform is None:
            print("⚠ midas_transform is None!")
            return None
            
        input_batch = midas_transform(rgb_frame)
        if torch.backends.mps.is_available():
            input_batch = input_batch.to("mps")
        
        with torch.no_grad():
            prediction = depth_model(input_batch)
            depth_map = prediction.squeeze().cpu().numpy()
        
        return depth_map
    except Exception as e:
        print(f"❌ Depth estimation error: {e}")
        import traceback
        traceback.print_exc()
        return None

def combine_keypoints_with_depth(keypoints_2d, depth_map, frame_shape):
    """
    Combine 2D keypoints with depth map to get 3D keypoints.
    
    Uses bilinear interpolation for better accuracy when mapping from
    high-resolution frame to lower-resolution depth map.
    """
    if depth_map is None or keypoints_2d is None:
        return None
    
    keypoints_3d = []
    frame_h, frame_w = frame_shape[:2]
    depth_h, depth_w = depth_map.shape
    
    for kp in keypoints_2d:
        x, y, conf = kp[0], kp[1], kp[2]
        
        if conf > 0.5:  # Only if keypoint is visible
            # Map keypoint coordinates to depth map coordinates (as float for interpolation)
            depth_x = x * depth_w / frame_w
            depth_y = y * depth_h / frame_h
            
            # Get integer coordinates for interpolation
            x0 = int(depth_x)
            y0 = int(depth_y)
            x1 = min(x0 + 1, depth_w - 1)
            y1 = min(y0 + 1, depth_h - 1)
            
            # Clamp to valid range
            x0 = max(0, min(depth_w - 1, x0))
            y0 = max(0, min(depth_h - 1, y0))
            
            # Bilinear interpolation weights
            wx = depth_x - x0
            wy = depth_y - y0
            
            # Sample 4 surrounding pixels
            z00 = depth_map[y0, x0]
            z01 = depth_map[y0, x1]
            z10 = depth_map[y1, x0]
            z11 = depth_map[y1, x1]
            
            # Bilinear interpolation for smoother depth value
            z = (1 - wx) * (1 - wy) * z00 + \
                wx * (1 - wy) * z01 + \
                (1 - wx) * wy * z10 + \
                wx * wy * z11
            
            keypoints_3d.append([x, y, z, conf])
        else:
            keypoints_3d.append([x, y, 0.0, conf])  # No depth if not visible
    
    return np.array(keypoints_3d)

def calculate_shoulder_roll(shoulder_3d, elbow_3d, wrist_3d=None):
    """
    Calculate shoulder roll angle (towards/away from camera).
    
    Uses wrist if available (more accurate), falls back to elbow.
    
    Args:
        shoulder_3d: [x, y, z, confidence] for shoulder
        elbow_3d: [x, y, z, confidence] for elbow
        wrist_3d: [x, y, z, confidence] for wrist (optional, preferred)
    
    Returns:
        roll angle in degrees:
        - 0°: Arm pointing away from camera (straight back)
        - 90°: Arm pointing right (perpendicular)
        - 180°: Arm pointing towards camera (straight forward)
        - Negative angles for left direction
    
    Interpretation:
        - dz < 0: Arm moving TOWARDS camera (outward/forward)
        - dz > 0: Arm moving AWAY from camera (inward/backward)
    """
    if shoulder_3d[3] < 0.5:
        return None
    
    # Prefer wrist (more accurate - larger depth difference)
    # Fall back to elbow if wrist not available
    if wrist_3d is not None and wrist_3d[3] > 0.5:
        end_point = wrist_3d
        point_name = "wrist"
    elif elbow_3d[3] > 0.5:
        end_point = elbow_3d
        point_name = "elbow"
    else:
        return None
    
    # Project to XZ plane (top view)
    dx = end_point[0] - shoulder_3d[0]  # Horizontal (left/right)
    dz = end_point[2] - shoulder_3d[2]  # Depth (towards/away)
    
    # Calculate angle in XZ plane
    roll = np.degrees(np.arctan2(dx, dz))
    
    return roll

def draw_data_overlay(frame, keypoints, boxes, person_idx=0, keypoints_3d=None):
    """Draw additional pose data on the frame including depth values"""
    if keypoints is None or len(keypoints) == 0:
        return frame
    
    person_kp = keypoints[person_idx]
    # keypoints_3d is already for one person (shape: 17, 4), not a list of people
    # Ensure it's a numpy array for consistent indexing
    if keypoints_3d is not None:
        person_kp_3d = np.asarray(keypoints_3d)
        # Ensure it's 2D (17, 4)
        if person_kp_3d.ndim == 1:
            person_kp_3d = person_kp_3d.reshape(-1, 4)
    else:
        person_kp_3d = None
    bbox = boxes[person_idx] if boxes is not None and len(boxes) > person_idx else None
    
    # Get frame dimensions
    h, w = frame.shape[:2]
    
    # ============================================
    # Draw bounding box info
    # ============================================
    if bbox is not None:
        x1, y1, x2, y2, conf, cls = bbox
        # Draw confidence on bounding box
        cv2.putText(frame, f"Conf: {conf:.2f}", 
                   (int(x1), int(y1) - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    
    # ============================================
    # Draw joint angles
    # ============================================
    angles = {}
    
    # Right elbow angle
    if len(person_kp) >= 11:
        right_elbow_angle = calculate_angle(person_kp[6], person_kp[8], person_kp[10])
        if right_elbow_angle is not None:
            angles['right_elbow'] = right_elbow_angle
            # Draw angle text near elbow
            elbow_pos = (int(person_kp[8][0]), int(person_kp[8][1]))
            cv2.putText(frame, f"{right_elbow_angle:.0f}°", 
                       (elbow_pos[0] + 10, elbow_pos[1]),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
            # Draw angle arc
            cv2.ellipse(frame, elbow_pos, (20, 20), 0, 0, int(right_elbow_angle),
                       (255, 0, 255), 2)
    
    # Left elbow angle
    if len(person_kp) >= 10:
        left_elbow_angle = calculate_angle(person_kp[5], person_kp[7], person_kp[9])
        if left_elbow_angle is not None:
            angles['left_elbow'] = left_elbow_angle
            elbow_pos = (int(person_kp[7][0]), int(person_kp[7][1]))
            cv2.putText(frame, f"{left_elbow_angle:.0f}°", 
                       (elbow_pos[0] - 50, elbow_pos[1]),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
            cv2.ellipse(frame, elbow_pos, (20, 20), 0, 0, int(left_elbow_angle),
                       (255, 0, 255), 2)
    
    # Right knee angle
    if len(person_kp) >= 17:
        right_knee_angle = calculate_angle(person_kp[12], person_kp[14], person_kp[16])
        if right_knee_angle is not None:
            angles['right_knee'] = right_knee_angle
            knee_pos = (int(person_kp[14][0]), int(person_kp[14][1]))
            cv2.putText(frame, f"{right_knee_angle:.0f}°", 
                       (knee_pos[0] + 10, knee_pos[1]),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    
    # Left knee angle
    if len(person_kp) >= 16:
        left_knee_angle = calculate_angle(person_kp[11], person_kp[13], person_kp[15])
        if left_knee_angle is not None:
            angles['left_knee'] = left_knee_angle
            knee_pos = (int(person_kp[13][0]), int(person_kp[13][1]))
            cv2.putText(frame, f"{left_knee_angle:.0f}°", 
                       (knee_pos[0] - 50, knee_pos[1]),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    
    # ============================================
    # Draw keypoint depth values and confidence
    # ============================================
    for kp_idx, (x, y, conf) in enumerate(person_kp):
        if conf > 0.3:  # Show info for visible keypoints
            kp_pos = (int(x), int(y))
            
            # Get depth value if available
            depth_val = None
            if person_kp_3d is not None and kp_idx < person_kp_3d.shape[0]:
                try:
                    # Use proper numpy array indexing: [row, col]
                    conf_3d = float(person_kp_3d[kp_idx, 3])
                    if conf_3d > 0.5:
                        depth_val = float(person_kp_3d[kp_idx, 2])
                except (IndexError, TypeError, ValueError):
                    depth_val = None
            
            # Draw depth value (more prominent) and confidence
            if depth_val is not None:
                # Draw depth value in cyan (more visible)
                depth_text = f"D:{depth_val:.0f}"
                text_size = cv2.getTextSize(depth_text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
                # Background rectangle for better visibility
                cv2.rectangle(frame,
                            (kp_pos[0] - text_size[0]//2 - 2, kp_pos[1] - 20),
                            (kp_pos[0] + text_size[0]//2 + 2, kp_pos[1] - 5),
                            (0, 0, 0), -1)  # Black background
                cv2.putText(frame, depth_text,
                           (kp_pos[0] - text_size[0]//2, kp_pos[1] - 8),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 2)  # Cyan text
                
                # Draw keypoint name for important points
                if kp_idx in [5, 6, 7, 8]:  # Shoulders and elbows
                    kp_name = KEYPOINT_NAMES[kp_idx].replace('_', ' ').title()
                    name_size = cv2.getTextSize(kp_name, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)[0]
                    cv2.putText(frame, kp_name,
                               (kp_pos[0] - name_size[0]//2, kp_pos[1] - 35),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 200, 0), 1)
            else:
                # Just show confidence if no depth
                conf_text = f"{conf:.2f}"
                text_size = cv2.getTextSize(conf_text, cv2.FONT_HERSHEY_SIMPLEX, 0.3, 1)[0]
                cv2.putText(frame, conf_text,
                           (kp_pos[0] - text_size[0]//2, kp_pos[1] - 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)
    
    # ============================================
    # Draw depth-aware skeleton connections
    # ============================================
    # Define skeleton connections (for depth visualization)
    skeleton_connections = [
        (5, 7),   # Left shoulder to left elbow
        (7, 9),   # Left elbow to left wrist
        (6, 8),   # Right shoulder to right elbow
        (8, 10),  # Right elbow to right wrist
        (5, 6),   # Left shoulder to right shoulder
        (5, 11),  # Left shoulder to left hip
        (6, 12),  # Right shoulder to right hip
        (11, 12), # Left hip to right hip
    ]
    
    # Draw skeleton with depth coloring
    for start_idx, end_idx in skeleton_connections:
        if (start_idx < len(person_kp) and end_idx < len(person_kp) and
            person_kp[start_idx][2] > 0.5 and person_kp[end_idx][2] > 0.5):
            
            start_pos = (int(person_kp[start_idx][0]), int(person_kp[start_idx][1]))
            end_pos = (int(person_kp[end_idx][0]), int(person_kp[end_idx][1]))
            
            # Get depth values for color coding
            depth_start = None
            depth_end = None
            if person_kp_3d is not None:
                try:
                    if start_idx < person_kp_3d.shape[0]:
                        conf_start = float(person_kp_3d[start_idx, 3])
                        if conf_start > 0.5:
                            depth_start = float(person_kp_3d[start_idx, 2])
                    if end_idx < person_kp_3d.shape[0]:
                        conf_end = float(person_kp_3d[end_idx, 3])
                        if conf_end > 0.5:
                            depth_end = float(person_kp_3d[end_idx, 2])
                except (IndexError, TypeError, ValueError):
                    pass
            
            # Color based on depth (closer = brighter, farther = darker)
            if depth_start is not None and depth_end is not None:
                avg_depth = (depth_start + depth_end) / 2
                # Normalize depth to color (assuming depth range 0-1000)
                # Closer objects (lower depth) = brighter yellow/cyan
                # Farther objects (higher depth) = darker blue
                depth_normalized = min(1.0, max(0.0, avg_depth / 1000.0))
                if depth_normalized < 0.5:
                    # Close - bright cyan/yellow
                    color = (255 - int(depth_normalized * 200), 255, 255)
                else:
                    # Far - darker blue
                    color = (100, 100, 255 - int((depth_normalized - 0.5) * 100))
            else:
                # No depth - use default yellow
                color = (255, 255, 0)
            
            # Draw line with depth-based color
            cv2.line(frame, start_pos, end_pos, color, 2)
            
            # Draw depth value on the connection midpoint
            if depth_start is not None and depth_end is not None:
                mid_x = (start_pos[0] + end_pos[0]) // 2
                mid_y = (start_pos[1] + end_pos[1]) // 2
                avg_depth = (depth_start + depth_end) / 2
                depth_text = f"{avg_depth:.0f}"
                cv2.putText(frame, depth_text,
                           (mid_x, mid_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
    
    # ============================================
    # Draw distances between keypoints (legacy)
    # ============================================
    # Shoulder width
    if person_kp[5][2] > 0.5 and person_kp[6][2] > 0.5:
        shoulder_dist = calculate_distance(person_kp[5], person_kp[6])
        mid_shoulder = ((person_kp[5][0] + person_kp[6][0]) / 2,
                       (person_kp[5][1] + person_kp[6][1]) / 2)
        # Only draw if depth not available (to avoid clutter)
        if person_kp_3d is None:
            cv2.line(frame, 
                    (int(person_kp[5][0]), int(person_kp[5][1])),
                    (int(person_kp[6][0]), int(person_kp[6][1])),
                    (255, 255, 0), 2)
            cv2.putText(frame, f"{shoulder_dist:.0f}px",
                       (int(mid_shoulder[0]), int(mid_shoulder[1])),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
    
    # Hip width
    if len(person_kp) >= 13 and person_kp[11][2] > 0.5 and person_kp[12][2] > 0.5:
        hip_dist = calculate_distance(person_kp[11], person_kp[12])
        mid_hip = ((person_kp[11][0] + person_kp[12][0]) / 2,
                  (person_kp[11][1] + person_kp[12][1]) / 2)
        cv2.line(frame,
                (int(person_kp[11][0]), int(person_kp[11][1])),
                (int(person_kp[12][0]), int(person_kp[12][1])),
                (255, 255, 0), 2)
        cv2.putText(frame, f"{hip_dist:.0f}px",
                   (int(mid_hip[0]), int(mid_hip[1])),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
    
    # ============================================
    # Draw data panel on the side
    # ============================================
    panel_x = w - 280
    panel_y = 10
    panel_height = 250 if person_kp_3d is not None else 200
    
    # Draw semi-transparent panel background
    overlay = frame.copy()
    cv2.rectangle(overlay, (panel_x, panel_y), (w - 10, panel_y + panel_height),
                 (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
    
    # Draw text on panel
    y_offset = 30
    cv2.putText(frame, "POSE DATA", (panel_x + 10, panel_y + y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    y_offset += 25
    
    # Depth info section
    if person_kp_3d is not None:
        cv2.putText(frame, "DEPTH INFO:", (panel_x + 10, panel_y + y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        y_offset += 20
        
        # Count keypoints with depth
        visible_with_depth = 0
        depth_values = []
        if person_kp_3d is not None:
            try:
                for i in range(person_kp_3d.shape[0]):
                    conf_3d = float(person_kp_3d[i, 3])
                    depth_3d = float(person_kp_3d[i, 2])
                    if conf_3d > 0.5 and depth_3d > 0:
                        visible_with_depth += 1
                        depth_values.append(depth_3d)
            except (IndexError, TypeError, ValueError):
                pass
        cv2.putText(frame, f"3D Points: {visible_with_depth}/17", 
                   (panel_x + 10, panel_y + y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        y_offset += 18
        
        # Show depth range for visible points
        if depth_values:
            min_depth = min(depth_values)
            max_depth = max(depth_values)
            cv2.putText(frame, f"Range: {min_depth:.0f}-{max_depth:.0f}", 
                       (panel_x + 10, panel_y + y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            y_offset += 18
            
            # Depth legend
            cv2.putText(frame, "Legend:", (panel_x + 10, panel_y + y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            y_offset += 15
            cv2.putText(frame, "Cyan=Close", (panel_x + 10, panel_y + y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)
            y_offset += 15
            cv2.putText(frame, "Blue=Far", (panel_x + 10, panel_y + y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 100, 100), 1)
            y_offset += 20
    
    # Visible keypoints count
    visible_count = sum(1 for kp in person_kp if kp[2] > 0.5)
    cv2.putText(frame, f"Visible KP: {visible_count}/17", 
               (panel_x + 10, panel_y + y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    y_offset += 20
    
    # Joint angles
    if angles:
        cv2.putText(frame, "Joint Angles:", (panel_x + 10, panel_y + y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        y_offset += 20
        for joint, angle in angles.items():
            if angle is not None:
                cv2.putText(frame, f"  {joint}: {angle:.0f}°",
                           (panel_x + 10, panel_y + y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 255), 1)
                y_offset += 18
    
    return frame

def print_pose_data(frame_num, keypoints, boxes, person_idx=0):
    """Print all pose estimation data to terminal"""
    if keypoints is None or len(keypoints) == 0:
        print(f"\n[Frame {frame_num}] No person detected")
        return
    
    person_kp = keypoints[person_idx]
    bbox = boxes[person_idx] if boxes is not None and len(boxes) > person_idx else None
    
    # Clear screen (optional - comment out if you want to see history)
    # print("\033[2J\033[H")  # Uncomment to clear screen each frame
    
    print("\n" + "="*80)
    print(f"FRAME {frame_num} - POSE ESTIMATION DATA")
    print("="*80)
    
    # Bounding box info
    if bbox is not None:
        x1, y1, x2, y2, conf, cls = bbox
        print(f"\n📦 BOUNDING BOX:")
        print(f"   Position: ({x1:.1f}, {y1:.1f}) to ({x2:.1f}, {y2:.1f})")
        print(f"   Size: {x2-x1:.1f} x {y2-y1:.1f} pixels")
        print(f"   Confidence: {conf:.3f}")
        print(f"   Class: {cls} (person)")
    
    # Keypoints
    print(f"\n📍 KEYPOINTS (17 total):")
    visible_count = 0
    for kp_idx, (x, y, conf) in enumerate(person_kp):
        kp_name = KEYPOINT_NAMES[kp_idx]
        visible = "✓" if conf > 0.5 else "✗"
        status = "VISIBLE" if conf > 0.5 else "HIDDEN"
        visible_count += 1 if conf > 0.5 else 0
        
        print(f"   {kp_idx:2d}. {kp_name:15s}: ({x:6.1f}, {y:6.1f}) | Conf: {conf:.3f} {visible} [{status}]")
    
    print(f"\n   Summary: {visible_count}/17 keypoints visible")
    
    # Calculate and print joint angles
    print(f"\n🦴 JOINT ANGLES:")
    angles = {}
    
    # Right elbow
    if len(person_kp) >= 11:
        right_elbow_angle = calculate_angle(person_kp[6], person_kp[8], person_kp[10])
        if right_elbow_angle is not None:
            angles['right_elbow'] = right_elbow_angle
            print(f"   Right Elbow:  {right_elbow_angle:.1f}° (shoulder→elbow→wrist)")
    
    # Left elbow
    if len(person_kp) >= 10:
        left_elbow_angle = calculate_angle(person_kp[5], person_kp[7], person_kp[9])
        if left_elbow_angle is not None:
            angles['left_elbow'] = left_elbow_angle
            print(f"   Left Elbow:   {left_elbow_angle:.1f}° (shoulder→elbow→wrist)")
    
    # Right knee
    if len(person_kp) >= 17:
        right_knee_angle = calculate_angle(person_kp[12], person_kp[14], person_kp[16])
        if right_knee_angle is not None:
            angles['right_knee'] = right_knee_angle
            print(f"   Right Knee:   {right_knee_angle:.1f}° (hip→knee→ankle)")
    
    # Left knee
    if len(person_kp) >= 16:
        left_knee_angle = calculate_angle(person_kp[11], person_kp[13], person_kp[15])
        if left_knee_angle is not None:
            angles['left_knee'] = left_knee_angle
            print(f"   Left Knee:    {left_knee_angle:.1f}° (hip→knee→ankle)")
    
    if not angles:
        print("   (No angles calculated - insufficient visible keypoints)")
    
    # Calculate and print distances
    print(f"\n📏 DISTANCES:")
    distances = {}
    
    # Shoulder width
    if person_kp[5][2] > 0.5 and person_kp[6][2] > 0.5:
        shoulder_dist = calculate_distance(person_kp[5], person_kp[6])
        distances['shoulder_width'] = shoulder_dist
        print(f"   Shoulder Width: {shoulder_dist:.1f} pixels")
    
    # Hip width
    if len(person_kp) >= 13 and person_kp[11][2] > 0.5 and person_kp[12][2] > 0.5:
        hip_dist = calculate_distance(person_kp[11], person_kp[12])
        distances['hip_width'] = hip_dist
        print(f"   Hip Width:     {hip_dist:.1f} pixels")
    
    # Arm lengths
    if len(person_kp) >= 11:
        if person_kp[6][2] > 0.5 and person_kp[10][2] > 0.5:
            right_arm_length = calculate_distance(person_kp[6], person_kp[10])
            distances['right_arm'] = right_arm_length
            print(f"   Right Arm:     {right_arm_length:.1f} pixels (shoulder→wrist)")
        
        if person_kp[5][2] > 0.5 and person_kp[9][2] > 0.5:
            left_arm_length = calculate_distance(person_kp[5], person_kp[9])
            distances['left_arm'] = left_arm_length
            print(f"   Left Arm:      {left_arm_length:.1f} pixels (shoulder→wrist)")
    
    # Torso height
    if len(person_kp) >= 13:
        if person_kp[5][2] > 0.5 and person_kp[11][2] > 0.5:
            torso_height = calculate_distance(person_kp[5], person_kp[11])
            distances['torso_height'] = torso_height
            print(f"   Torso Height:  {torso_height:.1f} pixels (shoulder→hip)")
    
    if not distances:
        print("   (No distances calculated - insufficient visible keypoints)")
    
    # Summary statistics
    print(f"\n📊 STATISTICS:")
    avg_confidence = np.mean([kp[2] for kp in person_kp if kp[2] > 0.5]) if visible_count > 0 else 0
    print(f"   Visible Keypoints: {visible_count}/17 ({visible_count/17*100:.1f}%)")
    print(f"   Avg Confidence:    {avg_confidence:.3f}")
    if angles:
        print(f"   Joint Angles:     {len(angles)} calculated")
    if distances:
        print(f"   Distances:        {len(distances)} calculated")
    
    print("="*80)

frame_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame from webcam.")
        break

    frame_count += 1
    
    # Run pose estimation
    results = model.predict(source=frame, device=device, verbose=False)
    result = results[0]
    
    # Get keypoints and boxes data
    keypoints = result.keypoints.data.cpu().numpy() if result.keypoints is not None else None
    boxes = result.boxes.data.cpu().numpy() if result.boxes is not None else None
    
    # Estimate depth map (every 5 frames to reduce computation)
    depth_map = None
    keypoints_3d = None
    
    if DEPTH_AVAILABLE and frame_count % 5 == 0:  # Run depth every 5 frames
        print("\n" + "="*60)
        print(f"🔍 Running depth estimation (frame {frame_count})...")
        print("="*60)
        depth_map = estimate_depth_map(frame)
        if depth_map is not None:
            print(f"✓ Depth map generated: shape {depth_map.shape}")
            print(f"   Depth range: {depth_map.min():.2f} to {depth_map.max():.2f}")
            print(f"   Note: MiDaS outputs relative depth values (not absolute distance)")
            if keypoints is not None and len(keypoints) > 0:
                keypoints_3d = combine_keypoints_with_depth(keypoints[0], depth_map, frame.shape)
                
                # Calculate 3D angles if we have depth
                if keypoints_3d is not None:
                    print(f"\n🎯 DEPTH ESTIMATION RESULTS (Frame {frame_count})")
                    print(f"📊 3D KEYPOINTS WITH DEPTH:")
                    # Show depth for visible keypoints
                    visible_with_depth = []
                    for i, kp_3d in enumerate(keypoints_3d):
                        if kp_3d[3] > 0.5:  # Visible
                            visible_with_depth.append((i, KEYPOINT_NAMES[i], kp_3d[2]))
                    if visible_with_depth:
                        for idx, name, depth_val in visible_with_depth[:7]:  # Show first 7
                            print(f"   {name:15s}: depth = {depth_val:.2f} (relative)")
                        if len(visible_with_depth) > 7:
                            print(f"   ... and {len(visible_with_depth) - 7} more")
                    print(f"\n🎯 3D ANGLES (with depth):")
                    
                    angles_printed = False
                    
                    # Right shoulder roll
                    if len(keypoints_3d) >= 9:
                        right_roll = calculate_shoulder_roll(keypoints_3d[6], keypoints_3d[8])
                        if right_roll is not None:
                            # Determine direction
                            end_point = right_wrist if right_wrist is not None and right_wrist[3] > 0.5 else keypoints_3d[8]
                            dz = end_point[2] - keypoints_3d[6][2]
                            direction = "towards camera (outward)" if dz < 0 else "away from camera (inward)"
                            point_used = "wrist" if right_wrist is not None and right_wrist[3] > 0.5 else "elbow"
                            print(f"   Right Shoulder Roll: {right_roll:.1f}° ({direction}, using {point_used})")
                            angles_printed = True
                        else:
                            print(f"   Right Shoulder Roll: (elbow/wrist not visible)")
                    
                    # Left shoulder roll (using wrist if available, else elbow)
                    if len(keypoints_3d) >= 10:
                        left_wrist = keypoints_3d[9] if keypoints_3d[9][3] > 0.5 else None
                        left_roll = calculate_shoulder_roll(keypoints_3d[5], keypoints_3d[7], left_wrist)
                        if left_roll is not None:
                            # Determine direction
                            end_point = left_wrist if left_wrist is not None and left_wrist[3] > 0.5 else keypoints_3d[7]
                            dz = end_point[2] - keypoints_3d[5][2]
                            direction = "towards camera (outward)" if dz < 0 else "away from camera (inward)"
                            point_used = "wrist" if left_wrist is not None and left_wrist[3] > 0.5 else "elbow"
                            print(f"   Left Shoulder Roll:  {left_roll:.1f}° ({direction}, using {point_used})")
                            angles_printed = True
                        else:
                            print(f"   Left Shoulder Roll:  (elbow/wrist not visible)")
                    
                    if not angles_printed:
                        print("   (No 3D angles calculated - elbows not visible enough)")
                    print("="*60 + "\n")
        elif frame_count == 5:
            print("⚠ Depth map generation returned None")
            print("   Check if depth_model and midas_transform are loaded correctly")
            print("="*60 + "\n")
    elif frame_count == 5:
        print("\n" + "="*60)
        print("⚠ Depth estimation skipped - DEPTH_AVAILABLE = False")
        print("   Depth model did not load successfully at startup")
        print("="*60 + "\n")
    
    # Print all data to terminal
    print_pose_data(frame_count, keypoints, boxes, person_idx=0)
    
    # Plot keypoints and skeleton (default visualization)
    annotated = result.plot()
    
    # Add our custom data overlays (including depth visualization)
    if keypoints is not None and len(keypoints) > 0:
        annotated = draw_data_overlay(annotated, keypoints, boxes, person_idx=0, keypoints_3d=keypoints_3d)
    
    # Add frame counter
    cv2.putText(annotated, f"Frame: {frame_count}", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Draw depth status and info
    if DEPTH_AVAILABLE:
        if depth_map is not None and keypoints_3d is not None:
            # Depth is active - show detailed info
            cv2.putText(annotated, "Depth: ACTIVE", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # Show depth range
            depth_min = depth_map.min()
            depth_max = depth_map.max()
            cv2.putText(annotated, f"Range: {depth_min:.0f}-{depth_max:.0f}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            # Show number of keypoints with depth
            if keypoints_3d is not None and keypoints_3d.shape[0] > 0:
                try:
                    # keypoints_3d is already a numpy array of shape (17, 4)
                    visible_with_depth = sum(1 for i in range(keypoints_3d.shape[0]) 
                                            if keypoints_3d[i, 3] > 0.5 and keypoints_3d[i, 2] > 0)
                    cv2.putText(annotated, f"3D Keypoints: {visible_with_depth}/17", (10, 110),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                except (IndexError, TypeError, ValueError):
                    pass
        else:
            cv2.putText(annotated, "Depth: WAITING", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 165, 0), 2)
            cv2.putText(annotated, "(Runs every 5 frames)", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 165, 0), 1)
    
    cv2.imshow("YOLOv8 Pose Estimation - Press 'q' to quit", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
print("Webcam feed stopped.")