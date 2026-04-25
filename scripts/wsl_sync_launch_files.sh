#!/usr/bin/env bash
set -eo pipefail

cp -f /mnt/d/catkin_ws/src/swarm_bringup/launch/swarm_namespaced.launch /home/chen/catkin_ws/src/swarm_bringup/launch/swarm_namespaced.launch
cp -f /mnt/d/catkin_ws/src/swarm_bringup/launch/swarm_multi_uav.launch /home/chen/catkin_ws/src/swarm_bringup/launch/swarm_multi_uav.launch
cp -f /mnt/d/catkin_ws/src/swarm_bringup/launch/swarm_multi_uav_3.launch /home/chen/catkin_ws/src/swarm_bringup/launch/swarm_multi_uav_3.launch
cp -f /mnt/d/catkin_ws/src/swarm_bringup/launch/swarm_multi_uav_6.launch /home/chen/catkin_ws/src/swarm_bringup/launch/swarm_multi_uav_6.launch

echo "LAUNCH_FILES_SYNCED"

