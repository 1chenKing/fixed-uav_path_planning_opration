#!/usr/bin/env bash
set -euo pipefail

export ROS_DISTRO=noetic
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
source /opt/ros/noetic/setup.bash
source /home/chen/catkin_ws/devel/setup.bash

echo "=== uav_1 state ==="
timeout 8s rostopic echo /uav_1/mavros/state -n 1 || true
echo "=== uav_2 state ==="
timeout 8s rostopic echo /uav_2/mavros/state -n 1 || true
echo "=== uav_1 extended_state ==="
timeout 8s rostopic echo /uav_1/mavros/extended_state -n 1 || true
echo "=== uav_1 local_position hz ==="
timeout 8s bash -lc 'rostopic hz /uav_1/mavros/local_position/pose | head -n 5' || true
