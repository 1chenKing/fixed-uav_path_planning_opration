#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"

if [ -d "$PX4_DIR/build/px4_sitl_default/bin" ]; then
  echo "PX4_SITL_BUILD=1"
  ls -la "$PX4_DIR/build/px4_sitl_default/bin"
else
  echo "PX4_SITL_BUILD=0"
fi

