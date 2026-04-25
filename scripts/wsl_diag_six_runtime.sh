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
bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 6 -w empty >/tmp/codex_six_fw.log 2>&1 &
FW_PID=$!

sleep 22

cd /home/chen/catkin_ws
roslaunch swarm_bringup swarm_multi_uav_6.launch >/tmp/codex_six_ros.log 2>&1 &
ROS_PID=$!

sleep 20

echo "=== STATE uav_1 ==="
timeout 12s rostopic echo /uav_1/mavros/state -n 1 || true
echo "=== GLOBAL uav_1 ==="
timeout 12s rostopic echo /uav_1/mavros/global_position/global -n 1 || true
echo "=== LOCAL uav_1 ==="
timeout 12s rostopic echo /uav_1/mavros/local_position/pose -n 1 || true
echo "=== EXTENDED uav_1 ==="
timeout 12s rostopic echo /uav_1/mavros/extended_state -n 1 || true
echo "=== MISSION SERVICES uav_1 ==="
rosservice list | grep "/uav_1/mavros/mission" || true
echo "=== MODE SERVICES uav_1 ==="
rosservice list | grep -E "/uav_1/mavros/(set_mode|cmd/arming)" || true
echo "=== UDP LISTENERS ==="
ss -lunp | grep -E "1454|1456|1458|px4" || true
echo "=== MAVROS CONNECTION LINES ==="
grep -E "MAVROS started|TARGET ID|Remote address|udp0" /tmp/codex_six_ros.log | tail -n 60 || true
echo "=== ROS LOG TAIL ==="
tail -n 120 /tmp/codex_six_ros.log || true
echo "=== FW LOG TAIL ==="
tail -n 80 /tmp/codex_six_fw.log || true
for rootfs_dir in /home/chen/catkin_ws/PX4_Firmware/build/px4_sitl_default/rootfs/*; do
  if [ -f "$rootfs_dir/log/last_log.txt" ]; then
    echo "=== ROOTFS LOG $rootfs_dir ==="
    tail -n 40 "$rootfs_dir/log/last_log.txt" || true
  fi
done

kill "$ROS_PID" || true
kill "$FW_PID" || true
cleanup
