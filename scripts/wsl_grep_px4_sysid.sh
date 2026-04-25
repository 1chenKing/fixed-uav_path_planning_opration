#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"

grep -R -nE "SYSID_THISMAV|MAV_SYS_ID|param set.*SYSID|param set-default.*SYSID|instance_to_sys_id" \
  "$PX4_DIR/ROMFS" \
  "$PX4_DIR/src" \
  "$PX4_DIR/platforms" \
  "$PX4_DIR/Tools" 2>/dev/null || true
