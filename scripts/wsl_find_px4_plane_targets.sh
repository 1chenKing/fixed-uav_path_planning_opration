#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"

echo "--- plane-related make targets ---"
grep -R -n "gazebo-classic.*plane\|plane$" "$PX4_DIR/Makefile" "$PX4_DIR/boards" "$PX4_DIR/platforms" 2>/dev/null | sed -n '1,160p'

