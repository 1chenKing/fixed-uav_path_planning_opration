#!/usr/bin/env bash
set -eo pipefail

export ROS_DISTRO=noetic
source /opt/ros/noetic/setup.bash
mkdir -p "$HOME/catkin_ws/src"
cd "$HOME/catkin_ws/src"

if [ ! -f CMakeLists.txt ]; then
  cat > CMakeLists.txt <<'EOF'
cmake_minimum_required(VERSION 3.0.2)
include(/opt/ros/noetic/share/catkin/cmake/toplevel.cmake)
EOF
fi

echo "CATKIN_SRC_READY=$HOME/catkin_ws/src"
ls -la "$HOME/catkin_ws/src"
