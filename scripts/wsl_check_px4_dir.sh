#!/usr/bin/env bash
set -euo pipefail

for path in \
  /home/chen/catkin_ws/PX4_Firmware \
  /home/chen/catkin_ws/PX4-Autopilot \
  /home/chen/catkin_ws/code
do
  if [ -d "$path" ]; then
    echo "FOUND=$path"
  else
    echo "MISSING=$path"
  fi
done

find /home/chen/catkin_ws -maxdepth 2 -type d \( -name PX4_Firmware -o -name PX4-Autopilot \) -print
