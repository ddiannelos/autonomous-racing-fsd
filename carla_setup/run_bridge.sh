#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Cleanup Function
cleanup() {
	echo "Caught signal, running unfreeze..."
	python3 "$SCRIPT_DIR/unfreeze_carla.py"
	exit
}

# Execute cleanup function on SIGINT or SIGTERM
trap cleanup SIGINT SIGTERM

# Source ROS 2 and install
if [ -z "$ROSDISTRO" ]; then
	source /opt/ros/humble/setup.bash
fi

# Define path for object files
OBJECT_FILE="$SCRIPT_DIR/objects.json"

# Run the launch command
echo "Starting CARLA ROS Bridge...."
source $SCRIPT_DIR/install/setup.bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py \
	town:=FormulaStadium \
	objects_definition_file:=$OBJECT_FILE \
	synchronous_mode:=true
