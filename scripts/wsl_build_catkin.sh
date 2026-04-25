#!/usr/bin/env bash
set -eo pipefail

export ROS_DISTRO=noetic
source /opt/ros/noetic/setup.bash
cd /home/chen/catkin_ws
mkdir -p /home/chen/catkin_ws/src
cat > /home/chen/catkin_ws/src/CMakeLists.txt <<'EOF'
cmake_minimum_required(VERSION 3.0.2)
include(/opt/ros/noetic/share/catkin/cmake/toplevel.cmake)
EOF
catkin_make
