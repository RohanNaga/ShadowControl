#!/bin/bash
# Quick start script for Shadow Control

set -e

echo "================================"
echo "Shadow Control - Quick Start"
echo "================================"
echo ""

# Source ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2..."
    source /opt/ros/humble/setup.bash
fi

# Source workspace
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    echo "Sourcing workspace..."
    source "$WORKSPACE_ROOT/install/setup.bash"
else
    echo "⚠️  Workspace not built yet!"
    echo "Run ./build_ros2.sh first"
    exit 1
fi

echo ""
echo "🚀 Starting Shadow Control system..."
echo ""

# Launch the system
ros2 launch shadow_control pose_estimation.launch.py
