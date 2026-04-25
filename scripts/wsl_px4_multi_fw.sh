#!/usr/bin/env bash
set -eo pipefail

PX4_DIR="${PX4_DIR:-/home/chen/catkin_ws/PX4_Firmware}"
VEHICLE_COUNT="${VEHICLE_COUNT:-2}"
WORLD="${WORLD:-empty}"
MODEL="${MODEL:-plane}"

cd "$PX4_DIR"

if [ ! -f ./Tools/simulation/sitl_multiple_run.sh ]; then
  echo "PX4 multi-vehicle Gazebo script not found"
  exit 1
fi

echo "Launching PX4 multi-vehicle SITL from $PX4_DIR"
echo "Vehicle count: $VEHICLE_COUNT"
echo "World: $WORLD"
echo "Model: $MODEL"

bash ./Tools/simulation/sitl_multiple_run.sh -m "$MODEL" -n "$VEHICLE_COUNT" -w "$WORLD"
