#!/usr/bin/env bash
set -eo pipefail

WS_DIR="${1:-$HOME/catkin_ws}"
PX4_DIR="${2:-$HOME/PX4-Autopilot}"

echo "[1/5] Workspace: $WS_DIR"
echo "[2/5] PX4 dir: $PX4_DIR"

if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo "ROS Noetic not found at /opt/ros/noetic/setup.bash"
  exit 1
fi

if [ ! -d "$WS_DIR/src" ]; then
  echo "Creating catkin source directory"
  mkdir -p "$WS_DIR/src"
fi

if [ ! -d "$PX4_DIR" ]; then
  echo "PX4 is not cloned yet."
  echo "Run:"
  echo "  git clone https://github.com/PX4/PX4-Autopilot.git --recursive $PX4_DIR"
  echo "  cd $PX4_DIR"
  echo "  bash ./Tools/setup/ubuntu.sh"
  exit 2
fi

echo "[3/5] Sourcing ROS"
export ROS_DISTRO=noetic
source /opt/ros/noetic/setup.bash

echo "[4/5] Building catkin workspace"
cd "$WS_DIR"
catkin_make

echo "[5/5] Done"
echo "source /opt/ros/noetic/setup.bash" 
echo "source $WS_DIR/devel/setup.bash"
