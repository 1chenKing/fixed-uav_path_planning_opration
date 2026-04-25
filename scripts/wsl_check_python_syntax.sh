#!/usr/bin/env bash
set -euo pipefail

python3 -m py_compile /home/chen/catkin_ws/src/mission_ui/src/mission_ui/swarm_control_plugin.py
python3 -m py_compile /home/chen/catkin_ws/src/avoidance_2d/scripts/avoidance_2d_node.py

echo "PYTHON_SYNTAX_OK"
