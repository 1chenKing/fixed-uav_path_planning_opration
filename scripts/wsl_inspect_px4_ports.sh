#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"

echo "--- sitl_multiple_run.sh ---"
grep -nE 'udp|MAV_SYS_ID|mavlink' "$PX4_DIR/Tools/simulation/sitl_multiple_run.sh" || true

echo "--- simulation port references ---"
grep -R -nE '1454|1456|mavlink_udp_port|MAV_SYS_ID' \
  "$PX4_DIR/Tools/simulation" \
  "$PX4_DIR/ROMFS" 2>/dev/null || true

