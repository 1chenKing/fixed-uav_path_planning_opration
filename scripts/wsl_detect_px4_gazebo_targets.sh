#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"

echo "--- gazebo classic targets ---"
grep -R -n "gazebo-classic_.*plane\|gazebo-classic_plane\|standard_vtol\|rc_cessna\|plane" \
  "$PX4_DIR"/boards "$PX4_DIR"/platforms "$PX4_DIR"/ROMFS "$PX4_DIR"/docs/en/sim_gazebo_classic 2>/dev/null | sed -n '1,220p'

