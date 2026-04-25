#!/usr/bin/env bash
set -eo pipefail

SCRIPT="/home/chen/catkin_ws/PX4_Firmware/Tools/simulation/gazebo-classic/sitl_multiple_run.sh"
sed -n '1,220p' "$SCRIPT"

