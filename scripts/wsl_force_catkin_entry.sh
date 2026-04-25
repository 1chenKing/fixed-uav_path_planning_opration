#!/usr/bin/env bash
set -euo pipefail

mkdir -p /home/chen/catkin_ws/src
cat > /home/chen/catkin_ws/src/CMakeLists.txt <<'EOF'
cmake_minimum_required(VERSION 3.0.2)
include(/opt/ros/noetic/share/catkin/cmake/toplevel.cmake)
EOF

echo "WROTE=/home/chen/catkin_ws/src/CMakeLists.txt"
ls -l /home/chen/catkin_ws/src/CMakeLists.txt
cat /home/chen/catkin_ws/src/CMakeLists.txt
