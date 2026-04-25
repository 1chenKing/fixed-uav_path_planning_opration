#!/usr/bin/env bash
set -eo pipefail

sudo apt update
sudo apt install -y \
  ros-noetic-mavros \
  ros-noetic-mavros-extras \
  ros-noetic-geographic-msgs \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-plugins \
  ros-noetic-rqt \
  ros-noetic-rqt-common-plugins \
  ros-noetic-rviz

sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

