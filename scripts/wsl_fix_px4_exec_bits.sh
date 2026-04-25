#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="/home/chen/catkin_ws/PX4_Firmware"

cd "$PX4_DIR"

find Tools -type f \( -name '*.sh' -o -name '*.py' \) -exec chmod +x {} +
find boards -type f -name '*.px4board' -exec chmod 644 {} + || true

if [ -f Tools/check_submodules.sh ]; then
  chmod +x Tools/check_submodules.sh
fi

echo "PX4 executable bits repaired where needed."

