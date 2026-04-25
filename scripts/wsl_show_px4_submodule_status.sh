#!/usr/bin/env bash
set -eo pipefail

cd /home/chen/catkin_ws/PX4_Firmware
git submodule status src/drivers/gps/devices || true

