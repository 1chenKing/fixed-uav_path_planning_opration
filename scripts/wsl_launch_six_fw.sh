#!/usr/bin/env bash
set -eo pipefail

export ROS_DISTRO=noetic
if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi

cd /home/chen/catkin_ws/PX4_Firmware

bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 6 -w empty
