#!/usr/bin/env bash
set -eo pipefail

export ROS_DISTRO=noetic
source /opt/ros/noetic/setup.bash

timeout 5s rostopic list | grep -E '^/uav_[12]/mavros|^/swarm/' || true

