#!/usr/bin/env bash
set -euo pipefail

export ROS_DISTRO=noetic
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash
source /home/chen/catkin_ws/devel/setup.bash

cleanup() {
  pkill -f "roslaunch swarm_bringup swarm_multi_uav_6.launch" || true
  pkill -f "sitl_multiple_run.sh -m plane -n 6 -w empty" || true
  pkill -x px4 || true
  pkill -x gzserver || true
  pkill -x gzclient || true
  pkill -x gazebo || true
}

cleanup

cd /home/chen/catkin_ws/PX4_Firmware
bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 6 -w empty >/tmp/codex_form_fw.log 2>&1 &
FW_PID=$!

sleep 22

cd /home/chen/catkin_ws
roslaunch swarm_bringup swarm_multi_uav_6.launch >/tmp/codex_form_ros.log 2>&1 &
ROS_PID=$!

sleep 20

rostopic pub -1 /swarm/formation_cmd swarm_msgs/FormationCommand "{formation_type: line, spacing: 26.0, heading_deg: 0.0, anchor: {header: {frame_id: map}, pose: {position: {x: 80.0, y: 60.0, z: 60.0}, orientation: {w: 1.0}}}, vehicle_ids: [uav_1, uav_2, uav_3, uav_4, uav_5, uav_6]}" >/tmp/codex_form_pub0.log 2>&1 || true
sleep 2
rostopic pub -1 /swarm/formation_cmd swarm_msgs/FormationCommand "{formation_type: v_shape, spacing: 32.0, heading_deg: 90.0, anchor: {header: {frame_id: map}, pose: {position: {x: 150.0, y: 120.0, z: 60.0}, orientation: {w: 1.0}}}, vehicle_ids: [uav_1, uav_2, uav_3, uav_4, uav_5, uav_6]}" >/tmp/codex_form_pub2.log 2>&1 || true
sleep 2
rostopic pub -1 /swarm/formation_cmd swarm_msgs/FormationCommand "{formation_type: echelon_left, spacing: 24.0, heading_deg: 135.0, anchor: {header: {frame_id: map}, pose: {position: {x: 210.0, y: 140.0, z: 60.0}, orientation: {w: 1.0}}}, vehicle_ids: [uav_1, uav_2, uav_3, uav_4, uav_5, uav_6]}" >/tmp/codex_form_pub3.log 2>&1 || true

sleep 3

echo "=== STATUS ==="
timeout 8s rostopic echo /swarm/status -n 1 || true
echo "=== SAFE CMD ==="
timeout 8s rostopic echo /swarm/formation_cmd_safe -n 1 || true
echo "=== DEBUG ANCHOR ==="
timeout 8s rostopic echo /swarm/debug_anchor -n 1 || true
echo "=== FORMATION MARKERS ==="
timeout 8s rostopic echo /swarm/formation_markers -n 1 | sed -n '1,80p' || true
echo "=== FORMATION LOG LINES ==="
grep -nE "Formation=|Safe formation=|Received formation command|formation_ready|Mission phase updated" /tmp/codex_form_ros.log | tail -n 80 || true

kill "$ROS_PID" || true
kill "$FW_PID" || true
cleanup
