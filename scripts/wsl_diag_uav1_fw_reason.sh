#!/usr/bin/env bash
set -euo pipefail

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
bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m plane -n 6 -w empty >/tmp/codex_fw_reason.log 2>&1 &
FW_PID=$!

sleep 35

echo "=== GREP ARM/PREFLIGHT/FAIL/COMMANDER ==="
grep -Ein "arm|preflight|fail|commander|denied|takeoff|runway|catapult" /tmp/codex_fw_reason.log | tail -n 200 || true

echo "=== ROOTFS FILES ==="
find /home/chen/catkin_ws/PX4_Firmware/build/px4_sitl_default/rootfs -maxdepth 2 -type f | grep -E "log|out|err" | sed -n '1,120p' || true

echo "=== FW LOG TAIL ==="
tail -n 200 /tmp/codex_fw_reason.log || true

kill "$FW_PID" || true
cleanup
