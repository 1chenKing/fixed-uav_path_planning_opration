#!/usr/bin/env bash
set -eo pipefail

export ROS_DISTRO=noetic
if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"
VEHICLE_COUNT="${VEHICLE_COUNT:-2}"
MODEL="${MODEL:-plane}"
WORLD="${WORLD:-empty}"

cd "$PX4_DIR"
bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m "$MODEL" -n "$VEHICLE_COUNT" -w "$WORLD"

