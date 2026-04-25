#!/usr/bin/env bash
set -eo pipefail

export ROS_DISTRO=noetic
source /opt/ros/noetic/setup.bash
source /home/chen/catkin_ws/devel/setup.bash

timeout 40s roslaunch swarm_bringup swarm_multi_uav_6.launch || test $? -eq 124

