#!/usr/bin/env bash
set -eo pipefail

export ROS_DISTRO=noetic
source /opt/ros/noetic/setup.bash
source /home/chen/catkin_ws/devel/setup.bash

cleanup() {
  pkill -x px4 || true
  pkill -x gzserver || true
  pkill -x gzclient || true
  pkill -f "sitl_multiple_run.sh" || true
  pkill -f "roslaunch swarm_bringup swarm_multi_uav.launch" || true
}

cleanup

cd /home/chen/catkin_ws/PX4_Firmware
bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 2 -w empty >/tmp/two_fw.log 2>&1 &
FW_PID=$!

sleep 12

timeout 35s roslaunch swarm_bringup swarm_multi_uav.launch || test $? -eq 124

kill "$FW_PID" || true
cleanup

