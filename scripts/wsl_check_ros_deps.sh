#!/usr/bin/env bash
set -eo pipefail

export ROS_DISTRO=noetic
source /opt/ros/noetic/setup.bash

for pkg in mavros geographic_msgs gazebo_ros rospy; do
  if rospack find "$pkg" >/dev/null 2>&1; then
    echo "$pkg=OK"
  else
    echo "$pkg=MISSING"
  fi
done

if command -v rqt >/dev/null 2>&1; then
  echo "rqt=OK"
else
  echo "rqt=MISSING"
fi
