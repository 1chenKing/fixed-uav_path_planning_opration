#!/usr/bin/env bash
set -euo pipefail

echo "SRC_DIR=/home/chen/catkin_ws/src"
if [ -e /home/chen/catkin_ws/src/CMakeLists.txt ]; then
  echo "SRC_CMAKELISTS_EXISTS=1"
  ls -l /home/chen/catkin_ws/src/CMakeLists.txt
else
  echo "SRC_CMAKELISTS_EXISTS=0"
fi

if [ -e /opt/ros/noetic/share/catkin/cmake/toplevel.cmake ]; then
  echo "TOPLEVEL_EXISTS=1"
  ls -l /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
else
  echo "TOPLEVEL_EXISTS=0"
fi

rm -rf /tmp/catkin_probe
if cmake -S /home/chen/catkin_ws/src -B /tmp/catkin_probe >/tmp/catkin_probe.log 2>&1; then
  echo "CMAKE_PROBE=OK"
else
  echo "CMAKE_PROBE=FAIL"
  cat /tmp/catkin_probe.log
fi
