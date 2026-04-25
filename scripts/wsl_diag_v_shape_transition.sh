#!/usr/bin/env bash
set -euo pipefail

export ROS_DISTRO=noetic
export ROS_MASTER_URI=http://localhost:11311
export QT_QPA_PLATFORM=offscreen

source /opt/ros/noetic/setup.bash
source /home/chen/catkin_ws/devel/setup.bash

cleanup() {
  pkill -f "roscore" || true
  pkill -f "rosmaster" || true
}

cleanup
roscore >/tmp/codex_diag_vshape_roscore.log 2>&1 &
ROSCORE_PID=$!
sleep 3

cd /home/chen/catkin_ws
python3 /mnt/d/catkin_ws/scripts/diag_v_shape_transition.py

kill "$ROSCORE_PID" || true
cleanup
