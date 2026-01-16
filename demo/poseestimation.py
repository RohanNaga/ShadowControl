import cv2
import mediapipe as mp
import numpy as np
import math

mp_pose = mp.solutions.pose
pose = mp_pose.Pose(static_image_mode=False,
                    model_complexity=1,
                    enable_segmentation=False,
                    min_detection_confidence=0.5,
                    min_tracking_confidence=0.5)

def angle_between(v1, v2):
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
    return np.degrees(np.arccos(dot))

def elbow_flexion(shoulder, elbow, wrist):
    return angle_between(shoulder - elbow, wrist - elbow)

def shoulder_abduction(hip, shoulder, elbow):
    torso = shoulder - hip
    arm = elbow - shoulder
    torso[2] = 0
    arm[2] = 0
    return angle_between(torso, arm)

def shoulder_extension(hip, shoulder, elbow):
    torso = shoulder - hip
    arm = elbow - shoulder
    torso[0] = 0
    arm[0] = 0
    return angle_between(torso, arm)

cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(image_rgb)

    if results.pose_landmarks:
        lm = results.pose_landmarks.landmark

        def p(i):
            return np.array([lm[i].x, lm[i].y, lm[i].z])

        L_SH, L_EL, L_WR, L_HIP = p(11), p(13), p(15), p(23)
        R_SH, R_EL, R_WR, R_HIP = p(12), p(14), p(16), p(24)

        left_elbow = elbow_flexion(L_SH, L_EL, L_WR)
        right_elbow = elbow_flexion(R_SH, R_EL, R_WR)

        left_abd = shoulder_abduction(L_HIP, L_SH, L_EL)
        right_abd = shoulder_abduction(R_HIP, R_SH, R_EL)

        left_ext = shoulder_extension(L_HIP, L_SH, L_EL)
        right_ext = shoulder_extension(R_HIP, R_SH, R_EL)

        print("\nLeft          |          Right")
        print(f"Shoulder abduction   {left_abd:6.1f}° | {right_abd:6.1f}°")
        print(f"Shoulder extension   {left_ext:6.1f}° | {right_ext:6.1f}°")
        print(f"Elbow flexion        {left_elbow:6.1f}° | {right_elbow:6.1f}°")

    cv2.imshow("Pose Estimation", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
