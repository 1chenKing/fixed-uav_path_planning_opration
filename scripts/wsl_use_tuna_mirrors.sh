#!/usr/bin/env bash
set -eo pipefail

UBUNTU_CODENAME="${UBUNTU_CODENAME:-focal}"
APT_FILE="/etc/apt/sources.list"
ROS_FILE="/etc/apt/sources.list.d/ros-latest.list"

sudo cp "$APT_FILE" "${APT_FILE}.bak.codex"

sudo tee "$APT_FILE" >/dev/null <<EOF
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ ${UBUNTU_CODENAME} main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ ${UBUNTU_CODENAME}-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ ${UBUNTU_CODENAME}-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ ${UBUNTU_CODENAME}-security main restricted universe multiverse
EOF

if [ -f "$ROS_FILE" ]; then
  sudo cp "$ROS_FILE" "${ROS_FILE}.bak.codex"
else
  sudo mkdir -p /etc/apt/sources.list.d
fi

sudo tee "$ROS_FILE" >/dev/null <<EOF
deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ ${UBUNTU_CODENAME} main
EOF

echo "APT mirror switched to Tsinghua Tuna for Ubuntu ${UBUNTU_CODENAME} and ROS."

