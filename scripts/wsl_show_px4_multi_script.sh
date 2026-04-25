#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"
SCRIPT="$PX4_DIR/Tools/simulation/sitl_multiple_run.sh"

echo "--- script head ---"
sed -n '1,220p' "$SCRIPT"

