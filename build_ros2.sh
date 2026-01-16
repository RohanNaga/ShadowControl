#!/bin/bash
# Build script for Shadow Control ROS2 package

set -e  # Exit on error

echo "================================"
echo "Shadow Control - ROS2 Build"
echo "================================"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2 not sourced. Sourcing /opt/ros/humble/setup.zsh..."
    source /opt/ros/humble/setup.zsh
fi

echo "✓ ROS2 $ROS_DISTRO detected"
echo ""

# Navigate to workspace root (parent of ShadowControl)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "📁 Workspace: $WORKSPACE_ROOT"
cd "$WORKSPACE_ROOT"

# Check if this is the first build
if [ ! -d "install" ]; then
    echo ""
    echo "🏗️  First time build - this may take a while..."
    echo ""
fi

# Build the package
echo "🔨 Building shadow_control package..."
colcon build --packages-select shadow_control

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Build successful!"
    echo ""
    echo "To use the package, run:"
    echo "  source install/setup.bash"
    echo ""
    echo "Then try:"
    echo "  ros2 run shadow_control pose_publisher"
    echo "  ros2 launch shadow_control pose_estimation.launch.py"
    echo ""
else
    echo ""
    echo "❌ Build failed!"
    exit 1
fi
