#!/bin/bash
set -e # Exit if any command fails

echo "Starting build process"

# 1. Source ROS2 (default is humble)
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2 humble..."
    source /opt/ros/humble/setup.bash
fi

# 2. Source CARLA ROS Bridge
CARLA_BRIDGE_DIR=${CARLA_ROS_BRIDGE_ROOT:-"$HOME/ros2_ws/carla-ros-bridge"}

if [ -f "$CARLA_BRIDGE_DIR/install/setup.bash" ]; then
    echo "Sourcing CARLA ROS Bridge from $CARLA_BRIDGE_DIR..."
    source "$CARLA_BRIDGE_DIR/install/setup.bash"
else
    echo "WARNING: CARLA ROS Bridge setup not found at $CARLA_BRIDGE_DIR."
    echo "If it is installed somewhere else run this before building:"
    echo "export CARLA_ROS_BRIDGE_ROOT=/path/to/your/carla-ros-bridge"
fi

# 3. Install dependencies
echo "Checking for missing ROS2 dependencies"
sudo apt-get update
rosdep update || echo "rosdep udpate failed (maybe already locked), continuing..."
rosdep install --from-paths src --ignore-src -r -y

# 4. Build the workspace
colcon build

echo "Build complete. To use the workspace, run the following in your terminal:"
echo "source install/setup.bash"