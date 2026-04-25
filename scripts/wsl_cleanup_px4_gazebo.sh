#!/usr/bin/env bash
set -eo pipefail

pkill -f "/home/chen/catkin_ws/PX4_Firmware/build/px4_sitl_default/bin/px4" || true
pkill -x px4 || true
pkill -x gzserver || true
pkill -x gzclient || true
pkill -x gazebo || true
pkill -f "sitl_run.sh" || true
pkill -f "gazebo-classic" || true

echo "PX4 and Gazebo-related processes cleaned."

