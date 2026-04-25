cmake_minimum_required(VERSION 3.0.2)
project(swarm_manager)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  swarm_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/swarm_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

