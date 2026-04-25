cmake_minimum_required(VERSION 3.0.2)
project(avoidance_2d)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  swarm_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/avoidance_2d_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
