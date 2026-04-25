#!/usr/bin/env bash
set -euo pipefail

cleanup() {
  pkill -f "make px4_sitl gazebo-classic_plane" || true
  pkill -x px4 || true
  pkill -x gzserver || true
  pkill -x gzclient || true
  pkill -x gazebo || true
}

cleanup

cd /home/chen/catkin_ws/PX4_Firmware
make px4_sitl gazebo-classic_plane >/tmp/single_fw_ports.log 2>&1 &
PX4_PID=$!

sleep 20

echo "=== px4 processes ==="
ps -ef | grep px4 | grep -v grep || true
echo "=== udp listeners ==="
ss -lunp | grep -E "145|px4" || true
echo "=== px4 log tail ==="
tail -n 120 /tmp/single_fw_ports.log || true

kill "$PX4_PID" || true
cleanup
