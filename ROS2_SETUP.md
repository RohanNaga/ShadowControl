# Shadow Control - ROS2 Setup Guide

## ROS2 Integration

This project uses **ROS2 Humble** for publish/subscribe communication between pose estimation and robot control.

### Architecture

```
┌─────────────────┐
│  Camera Input   │
└────────┬────────┘
         │
         ▼
┌─────────────────────────┐
│  Pose Publisher Node    │  ← MediaPipe processing
│  (pose_publisher.py)    │
└──────────┬──────────────┘
           │
           ├─► /shadow_control/pose (PoseData)
           └─► /shadow_control/pose_image (Image)
           
           ▼
    ┌──────────────┐
    │ Subscribers  │
    └──────────────┘
         │
         ├─► Robot Controller
         ├─► Data Logger
         ├─► Image Viewer
         └─► Custom Nodes
```

---

## Installation

### 1. Prerequisites

```bash
# ROS2 Humble (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop

# ROS2 development tools
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-cv-bridge

# Python dependencies
pip install opencv-python mediapipe numpy
```

### 2. Build the Package

```bash
# Navigate to workspace root
cd /home/revanth/revo/sc

# Source ROS2
source /opt/ros/humble/setup.bash

# Build the package
colcon build --packages-select shadow_control

# Source the workspace
source install/setup.bash
```

### 3. Verify Installation

```bash
# Check if package is found
ros2 pkg list | grep shadow_control

# Check available nodes
ros2 pkg executables shadow_control
```

---

## Usage

### Quick Start - Full System

```bash
# Terminal 1: Launch everything
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch shadow_control system.launch.py
```

### Individual Nodes

#### Pose Publisher (Main Node)

```bash
# Publish pose data from camera
ros2 run shadow_control pose_publisher

# With parameters
ros2 run shadow_control pose_publisher --ros-args \
  -p camera_id:=0 \
  -p publish_image:=true
```

#### Pose Subscriber (Debug/Testing)

```bash
# Print pose data to console
ros2 run shadow_control pose_subscriber
```

#### Image Viewer

```bash
# Display annotated video stream
ros2 run shadow_control image_viewer
```

#### Robot Controller

```bash
# Control robot from pose data (template)
ros2 run shadow_control robot_controller
```

### Launch Files

```bash
# Pose estimation only
ros2 launch shadow_control pose_estimation.launch.py

# With custom camera
ros2 launch shadow_control pose_estimation.launch.py camera_id:=0

# Without image display
ros2 launch shadow_control pose_estimation.launch.py show_image:=false

# Complete system (pose + robot control)
ros2 launch shadow_control system.launch.py
```

---

## Topics

### Published Topics

| Topic | Type | Description | Rate |
|-------|------|-------------|------|
| `/shadow_control/pose` | `shadow_control/PoseData` | Joint angles + landmarks | ~30 Hz |
| `/shadow_control/pose_image` | `sensor_msgs/Image` | Annotated video frame | ~30 Hz |

### Message Definitions

**PoseData.msg**
```
std_msgs/Header header
uint32 frame_id

shadow_control/JointAngles left
shadow_control/JointAngles right

shadow_control/Landmark3D left_shoulder
shadow_control/Landmark3D left_elbow
shadow_control/Landmark3D left_wrist
shadow_control/Landmark3D left_hip
shadow_control/Landmark3D right_shoulder
shadow_control/Landmark3D right_elbow
shadow_control/Landmark3D right_wrist
shadow_control/Landmark3D right_hip

float32 detection_confidence
bool pose_detected
```

**JointAngles.msg**
```
float32 shoulder_abduction  # degrees
float32 shoulder_extension  # degrees
float32 elbow_flexion       # degrees
```

**Landmark3D.msg**
```
float32 x           # normalized 0-1
float32 y           # normalized 0-1
float32 z           # depth
float32 visibility  # 0-1
```

---

## Monitoring & Debugging

### View Topics

```bash
# List all topics
ros2 topic list

# See pose data
ros2 topic echo /shadow_control/pose

# See message rate
ros2 topic hz /shadow_control/pose

# See topic info
ros2 topic info /shadow_control/pose
```

### View Nodes

```bash
# List running nodes
ros2 node list

# Node info
ros2 node info /pose_publisher
```

### RQt Tools

```bash
# Graph visualization
rqt_graph

# Runtime monitor
rqt

# Plot data
ros2 run rqt_plot rqt_plot /shadow_control/pose/left/elbow_flexion
```

---

## Creating Custom Subscribers

### Python Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from shadow_control.msg import PoseData

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            PoseData,
            '/shadow_control/pose',
            self.callback,
            10
        )
    
    def callback(self, msg):
        if msg.pose_detected:
            # Access joint angles
            left_elbow = msg.left.elbow_flexion
            right_elbow = msg.right.elbow_flexion
            
            # Access landmarks
            left_wrist_x = msg.left_wrist.x
            
            # Your custom logic here
            print(f"Elbows: L={left_elbow:.1f}° R={right_elbow:.1f}°")

def main():
    rclpy.init()
    node = MySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Add to Package

1. Save your node in `nodes/my_subscriber.py`
2. Add to `setup.py`:
   ```python
   'my_subscriber = nodes.my_subscriber:main',
   ```
3. Rebuild:
   ```bash
   colcon build --packages-select shadow_control
   source install/setup.bash
   ```
4. Run:
   ```bash
   ros2 run shadow_control my_subscriber
   ```

---

## Troubleshooting

### Camera Not Found

```bash
# List video devices
v4l2-ctl --list-devices

# Try different camera ID
ros2 run shadow_control pose_publisher --ros-args -p camera_id:=0
```

### Package Not Found After Build

```bash
# Make sure you sourced the workspace
source install/setup.bash

# Check if built correctly
ls install/shadow_control
```

### Message Type Errors

```bash
# Rebuild messages
colcon build --packages-select shadow_control --cmake-clean-cache
source install/setup.bash
```

### Import Errors (cv_bridge, etc.)

```bash
# Install missing ROS2 packages
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-vision-opencv
```

---

## Performance Tips

1. **Reduce Image Publishing**: Set `publish_image:=false` if you don't need visualization
2. **Lower Control Rate**: Adjust `control_rate_hz` parameter in robot controller
3. **QoS Settings**: Use `BEST_EFFORT` for real-time applications (modify node code)
4. **Process Priority**: Use `nice` or `chrt` for time-critical nodes

---

## Next Steps

1. Implement robot control logic in `robot_controller.py`
2. Add data logging subscriber for training data collection
3. Create configuration files for different robot setups
4. Add service calls for calibration and mode switching
5. Integrate with K-Scale robot interface

---

## References

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [MediaPipe Pose](https://google.github.io/mediapipe/solutions/pose)
- [cv_bridge Tutorial](http://wiki.ros.org/cv_bridge/Tutorials)
