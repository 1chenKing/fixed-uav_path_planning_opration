#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"

find "$PX4_DIR" -maxdepth 3 \( -name 'requirements*.txt' -o -name '*requirements*.txt' \) | sed -n '1,120p'

