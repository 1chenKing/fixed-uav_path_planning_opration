#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"

echo "--- instance dirs ---"
find "$PX4_DIR/build/px4_sitl_default" -maxdepth 2 -type d -name 'instance_*' | sed -n '1,40p'

echo "--- recent px4 logs ---"
find "$PX4_DIR/build/px4_sitl_default" -maxdepth 3 \( -name 'out.log' -o -name 'err.log' \) -printf '%TY-%Tm-%Td %TH:%TM:%TS %p\n' 2>/dev/null | sort | tail -n 20

