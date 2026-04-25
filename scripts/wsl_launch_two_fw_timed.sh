#!/usr/bin/env bash
set -eo pipefail

export ROS_DISTRO=noetic
if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi

cd /home/chen/catkin_ws/PX4_Firmware

timeout 45s bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 2 -w empty || test $? -eq 124

