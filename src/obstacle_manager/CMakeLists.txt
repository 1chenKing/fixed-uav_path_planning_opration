cmake_minimum_required(VERSION 3.0.2)
project(obstacle_manager)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  rospy
  swarm_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/obstacle_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
