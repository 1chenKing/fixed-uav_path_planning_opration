#!/usr/bin/env bash
set -eo pipefail

LAUNCH_FILE="/home/chen/catkin_ws/src/swarm_bringup/launch/swarm_multi_uav.launch"

echo "--- $LAUNCH_FILE ---"
cat "$LAUNCH_FILE"

