#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"

echo "PX4_DIR=$PX4_DIR"
echo "--- Tools ---"
ls "$PX4_DIR/Tools" | sed -n '1,80p'
echo "--- Matches ---"
find "$PX4_DIR" -maxdepth 3 \( -name '*multiple*' -o -name '*gazebo*' -o -name '*plane*' \) | sed -n '1,120p'

