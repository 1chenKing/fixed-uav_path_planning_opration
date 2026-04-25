#!/usr/bin/env bash
set -euo pipefail

SRC_DIR="/mnt/d/catkin_ws"
DST_DIR="$HOME/catkin_ws"

if [ ! -d "$SRC_DIR" ]; then
  echo "SOURCE_MISSING=$SRC_DIR" >&2
  exit 1
fi

mkdir -p "$DST_DIR"

if command -v rsync >/dev/null 2>&1; then
  rsync -a --delete \
    --exclude build \
    --exclude devel \
    --exclude install \
    --exclude .git \
    --exclude PX4_Firmware \
    "$SRC_DIR"/ "$DST_DIR"/
else
  cp -a "$SRC_DIR"/. "$DST_DIR"/
fi

echo "SYNCED_FROM=$SRC_DIR"
echo "SYNCED_TO=$DST_DIR"

if [ -f /opt/ros/noetic/setup.bash ]; then
  # Keep catkin workspace bootstrap file present even though it is not tracked in the Windows workspace.
  # Without this, rsync --delete removes src/CMakeLists.txt and breaks catkin_make in WSL.
  export ROS_DISTRO=noetic
  export ROS_MASTER_URI=http://localhost:11311
  source /opt/ros/noetic/setup.bash
  mkdir -p "$DST_DIR/src"
  if [ ! -f "$DST_DIR/src/CMakeLists.txt" ]; then
    cat > "$DST_DIR/src/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.0.2)
include(/opt/ros/noetic/share/catkin/cmake/toplevel.cmake)
EOF
  fi
fi
