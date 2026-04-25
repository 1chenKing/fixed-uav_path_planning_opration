#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"

python3 -m pip install --upgrade pip

if [ -f "$PX4_DIR/Tools/setup/requirements.txt" ]; then
  python3 -m pip install -r "$PX4_DIR/Tools/setup/requirements.txt"
fi

if [ -f "$PX4_DIR/src/modules/mavlink/mavlink/pymavlink/requirements.txt" ]; then
  python3 -m pip install -r "$PX4_DIR/src/modules/mavlink/mavlink/pymavlink/requirements.txt"
fi

