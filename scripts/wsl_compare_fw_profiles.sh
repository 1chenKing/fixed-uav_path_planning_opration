#!/usr/bin/env bash
set -euo pipefail

PX4_DIR=/home/chen/catkin_ws/PX4_Firmware

echo "=== AIRFRAME 4003 ==="
find "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes" -maxdepth 1 -type f | grep 4003 || true

echo "=== CESSNA MODELS ==="
find "$PX4_DIR" \( -iname "*rc_cessna*" -o -iname "*cessna*" \) | sed -n '1,80p'

echo "=== PLANE MODELS ==="
find "$PX4_DIR" \( -iname "*plane*" \) | sed -n '1,80p'

echo "=== AIRFRAME FILE SNIPPET ==="
AIRFRAME_FILE="$(find "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes" -maxdepth 1 -type f | grep 4003 | head -n 1 || true)"
if [ -n "${AIRFRAME_FILE}" ]; then
  sed -n '1,220p' "$AIRFRAME_FILE"
else
  echo "NO_4003_FILE"
fi

echo "=== 1030 GAZEBO-CLASSIC PLANE ==="
sed -n '1,220p' "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/1030_gazebo-classic_plane"

echo "=== 1032 GAZEBO-CLASSIC PLANE CATAPULT ==="
sed -n '1,220p' "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/1032_gazebo-classic_plane_catapult"
