#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"

find "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models" -maxdepth 2 \
  \( -iname '*plane*' -o -iname '*cessna*' \) | sed -n '1,120p'

