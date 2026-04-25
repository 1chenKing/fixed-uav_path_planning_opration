#!/usr/bin/env bash
set -eo pipefail

export ROS_DISTRO=noetic
source /opt/ros/noetic/setup.bash
source /home/chen/catkin_ws/devel/setup.bash

timeout 30s roslaunch swarm_bringup swarm_namespaced.launch vehicle_ns:=uav_1 fcu_url:=udp://:14540@127.0.0.1:14560 tgt_system:=1 || test $? -eq 124

