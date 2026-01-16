import cv2
import mediapipe as mp
import numpy as np
import math

mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles

pose = mp_pose.Pose(
    static_image_mode=False,
    model_complexity=1,
    enable_segmentation=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
)

def angle_between(v1, v2):
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-8 or n2 < 1e-8:
        return float("nan")
    v1 = v1 / n1
    v2 = v2 / n2
    dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
    return float(np.degrees(np.arccos(dot)))

def elbow_flexion(shoulder, elbow, wrist):
    return angle_between(shoulder - elbow, wrist - elbow)

def shoulder_abduction(hip, shoulder, elbow):
    torso = shoulder - hip
    arm = elbow - shoulder
    # project to XY (image plane)
    torso = torso.copy(); arm = arm.copy()
    torso[2] = 0
    arm[2] = 0
    return angle_between(torso, arm)

def shoulder_extension(hip, shoulder, elbow):
    torso = shoulder - hip
    arm = elbow - shoulder
    # project to YZ (sagittal-like)
    torso = torso.copy(); arm = arm.copy()
    torso[0] = 0
    arm[0] = 0
    return angle_between(torso, arm)

def to_px(lm, idx, w, h):
    """MediaPipe landmark -> pixel coords."""
    x = int(lm[idx].x * w)
    y = int(lm[idx].y * h)
    return x, y

def put_text_with_bg(img, text, org, font_scale=0.55, thickness=1, pad=4):
    """Readable text overlay (black bg, white text)."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = org
    # background box
    cv2.rectangle(img, (x - pad, y - th - pad), (x + tw + pad, y + baseline + pad), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)

cap = cv2.VideoCapture(1)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]

    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(image_rgb)

    if results.pose_landmarks:
        lm = results.pose_landmarks.landmark

        # Draw skeleton
        mp_draw.draw_landmarks(
            frame,
            results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=mp_styles.get_default_pose_landmarks_style(),
        )

        def p(i):
            return np.array([lm[i].x, lm[i].y, lm[i].z], dtype=np.float32)

        # 3D-ish normalized coords (mediapipe)
        L_SH, L_EL, L_WR, L_HIP = p(11), p(13), p(15), p(23)
        R_SH, R_EL, R_WR, R_HIP = p(12), p(14), p(16), p(24)

        left_elbow = elbow_flexion(L_SH, L_EL, L_WR)
        right_elbow = elbow_flexion(R_SH, R_EL, R_WR)

        left_abd = shoulder_abduction(L_HIP, L_SH, L_EL)
        right_abd = shoulder_abduction(R_HIP, R_SH, R_EL)

        left_ext = shoulder_extension(L_HIP, L_SH, L_EL)
        right_ext = shoulder_extension(R_HIP, R_SH, R_EL)

        # --- HUD (top-left) ---
        hud_lines = [
            "Angles (deg)",
            f"L Shoulder Abd: {left_abd:5.1f}   R: {right_abd:5.1f}",
            f"L Shoulder Ext: {left_ext:5.1f}   R: {right_ext:5.1f}",
            f"L Elbow Flex:   {left_elbow:5.1f}   R: {right_elbow:5.1f}",
            "ESC to quit",
        ]

        x0, y0 = 12, 24
        for i, line in enumerate(hud_lines):
            put_text_with_bg(frame, line, (x0, y0 + i * 22), font_scale=0.55, thickness=1)

        # --- Per-joint labels near shoulder/elbow (optional but useful) ---
        # Left shoulder + elbow
        l_sh_px = to_px(lm, 11, w, h)
        l_el_px = to_px(lm, 13, w, h)
        put_text_with_bg(frame, f"L Abd {left_abd:4.0f}", (l_sh_px[0] + 10, l_sh_px[1] - 10), font_scale=0.5)
        put_text_with_bg(frame, f"L Ext {left_ext:4.0f}", (l_sh_px[0] + 10, l_sh_px[1] + 12), font_scale=0.5)
        put_text_with_bg(frame, f"L Flex {left_elbow:4.0f}", (l_el_px[0] + 10, l_el_px[1] - 10), font_scale=0.5)

        # Right shoulder + elbow
        r_sh_px = to_px(lm, 12, w, h)
        r_el_px = to_px(lm, 14, w, h)
        put_text_with_bg(frame, f"R Abd {right_abd:4.0f}", (r_sh_px[0] + 10, r_sh_px[1] - 10), font_scale=0.5)
        put_text_with_bg(frame, f"R Ext {right_ext:4.0f}", (r_sh_px[0] + 10, r_sh_px[1] + 12), font_scale=0.5)
        put_text_with_bg(frame, f"R Flex {right_elbow:4.0f}", (r_el_px[0] + 10, r_el_px[1] - 10), font_scale=0.5)

    cv2.imshow("Pose Estimation", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
