#!/usr/bin/env bash
set -euo pipefail

echo "[1/6] Update apt index"
sudo apt update

echo "[2/6] Install base tools"
sudo apt install -y git python3-pip python3-rosdep python3-catkin-tools \
  python3-wstool build-essential ninja-build cmake

echo "[3/6] Install ROS and MAVROS dependencies"
sudo apt install -y ros-noetic-desktop-full ros-noetic-mavros ros-noetic-mavros-extras \
  ros-noetic-geographic-msgs ros-noetic-tf2-geometry-msgs ros-noetic-rviz \
  ros-noetic-rqt ros-noetic-rqt-common-plugins

echo "[4/6] Initialize rosdep if needed"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

echo "[5/6] Install Gazebo Classic"
sudo apt install -y gazebo11 libgazebo11-dev

echo "[6/6] Final note"
echo "Clone PX4-Autopilot separately inside WSL2, then install its Ubuntu dependencies."
echo "This script intentionally avoids cloning external repositories automatically."

