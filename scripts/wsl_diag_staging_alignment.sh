#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
source /home/chen/catkin_ws/devel/setup.bash

ROSCORE_PID=""
if ! timeout 3s rosnode list >/dev/null 2>&1; then
  roscore >/tmp/catkin_ws_diag_roscore.log 2>&1 &
  ROSCORE_PID="$!"
  sleep 4
fi
trap 'if [ -n "$ROSCORE_PID" ]; then kill "$ROSCORE_PID" >/dev/null 2>&1 || true; fi' EXIT

timeout 45s python3 /mnt/d/catkin_ws/scripts/diag_staging_alignment.py
