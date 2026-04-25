#!/usr/bin/env bash
set -euo pipefail

echo "=== pageFormation.py ==="
sed -n '1,280p' /home/chen/catkin_ws/code/Windows/groundControlPanel/pageFormation.py || true

echo
echo "=== cmd_process.py ==="
sed -n '1,280p' /home/chen/catkin_ws/code/WSL-Ubuntu22.04/drones-formation/cmd_process.py || true

echo
echo "=== config.py ==="
sed -n '1,240p' /home/chen/catkin_ws/code/WSL-Ubuntu22.04/drones-formation/config.py || true
