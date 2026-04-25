#!/usr/bin/env bash
set -euo pipefail

PX4_DIR=/home/chen/catkin_ws/PX4_Firmware

echo "PX4_DIR=$PX4_DIR"

for path in \
  "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_multiple_run.sh" \
  "$PX4_DIR/Tools/gazebo_sitl_multiple_run.sh" \
  "$PX4_DIR/Tools/sitl_multiple_run.sh"
do
  if [ -f "$path" ]; then
    echo "FOUND_FILE=$path"
  else
    echo "MISSING_FILE=$path"
  fi
done

find "$PX4_DIR/Tools" -maxdepth 4 -type f -name 'sitl_multiple_run.sh' -print 2>/dev/null || true
