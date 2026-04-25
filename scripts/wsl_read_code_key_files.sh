#!/usr/bin/env bash
set -euo pipefail

echo "=== start_sitl.sh ==="
sed -n '1,220p' /home/chen/catkin_ws/code/WSL-Ubuntu22.04/start_sitl.sh || true

echo
echo "=== bridge.py ==="
sed -n '1,260p' /home/chen/catkin_ws/code/WSL-Ubuntu22.04/drones-formation/bridge.py || true

echo
echo "=== groundMain.py ==="
sed -n '1,260p' /home/chen/catkin_ws/code/Windows/groundControlPanel/groundMain.py || true
