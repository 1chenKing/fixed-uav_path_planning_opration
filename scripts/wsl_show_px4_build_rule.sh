#!/usr/bin/env bash
set -eo pipefail

BUILD_FILE="/home/chen/catkin_ws/PX4_Firmware/build/px4_sitl_default/src/drivers/gps/CMakeFiles/git_gps_devices.dir/build.make"

if [ -f "$BUILD_FILE" ]; then
  grep -n "check_submodules\|git_init_devices" "$BUILD_FILE" || true
else
  echo "BUILD_RULE_MISSING"
fi

