#!/usr/bin/env bash
set -eo pipefail

cd /home/chen/catkin_ws/PX4_Firmware
./Tools/check_submodules.sh src/drivers/gps/devices

