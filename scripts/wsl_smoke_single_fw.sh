#!/usr/bin/env bash
set -eo pipefail

export ROS_DISTRO=noetic
if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi

cd /home/chen/catkin_ws/PX4_Firmware

timeout 180s make px4_sitl gazebo-classic_plane || test $? -eq 124

