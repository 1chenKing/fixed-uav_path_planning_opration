#!/usr/bin/env bash
set -eo pipefail

apt update
apt install -y \
  ros-noetic-rqt \
  ros-noetic-rqt-common-plugins

