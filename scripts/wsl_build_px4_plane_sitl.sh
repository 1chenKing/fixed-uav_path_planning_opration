#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"
TARGET="${1:-px4_sitl gazebo-classic_plane}"

export ROS_DISTRO=noetic
if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi

cd "$PX4_DIR"
make $TARGET
