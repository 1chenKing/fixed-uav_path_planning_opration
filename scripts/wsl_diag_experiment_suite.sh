#!/usr/bin/env bash
set -eo pipefail

cd /home/chen/catkin_ws
source /opt/ros/noetic/setup.bash
if [ -f devel/setup.bash ]; then
  source devel/setup.bash
fi

python3 /mnt/d/catkin_ws/scripts/diag_experiment_suite.py
