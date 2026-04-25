#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"
GAZEBO_BUILD_DIR="$PX4_DIR/build/px4_sitl_default/build_gazebo-classic"

if [ -d "$GAZEBO_BUILD_DIR" ]; then
  echo "GAZEBO_CLASSIC_BUILD_DIR=1"
  find "$GAZEBO_BUILD_DIR" -maxdepth 2 \( -name '*.so' -o -name 'gzserver' -o -name 'gazebo' \) | sed -n '1,120p'
else
  echo "GAZEBO_CLASSIC_BUILD_DIR=0"
fi

