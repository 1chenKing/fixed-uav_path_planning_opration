#!/usr/bin/env bash
set -e
cd /home/chen/catkin_ws
source /opt/ros/noetic/setup.bash
if [ -f /home/chen/catkin_ws/devel/setup.bash ]; then
  source /home/chen/catkin_ws/devel/setup.bash
fi
python3 /home/chen/catkin_ws/scripts/test_corridor_planner.py
