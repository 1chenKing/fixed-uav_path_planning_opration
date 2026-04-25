#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"

find "$PX4_DIR/Tools" -maxdepth 4 -type f \( -name '*multiple*run*.sh' -o -name '*multi*gazebo*.sh' -o -name '*sitl*multiple*.sh' \)

