# Complete pose estimation data
# Published by pose estimation node at camera frame rate

# Header with timestamp and frame info
std_msgs/Header header

# Frame sequence number
uint32 frame_id

# Joint angles for both sides
shadow_control/JointAngles left
shadow_control/JointAngles right

# Key landmarks (optional, for advanced control)
shadow_control/Landmark3D left_shoulder
shadow_control/Landmark3D left_elbow
shadow_control/Landmark3D left_wrist
shadow_control/Landmark3D left_hip
shadow_control/Landmark3D right_shoulder
shadow_control/Landmark3D right_elbow
shadow_control/Landmark3D right_wrist
shadow_control/Landmark3D right_hip

# Detection quality
float32 detection_confidence
bool pose_detected
