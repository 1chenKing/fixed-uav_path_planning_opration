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
  pkill -f "roslaunch swarm_bringup swarm_multi_uav_6.launch" || true
}

cleanup

cd /home/chen/catkin_ws/PX4_Firmware
bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 6 -w empty >/tmp/six_fw.log 2>&1 &
FW_PID=$!

sleep 18

timeout 45s roslaunch swarm_bringup swarm_multi_uav_6.launch || test $? -eq 124

kill "$FW_PID" || true
cleanup

