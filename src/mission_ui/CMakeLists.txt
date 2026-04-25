cmake_minimum_required(VERSION 3.0.2)
project(mission_ui)

find_package(catkin REQUIRED COMPONENTS
  qt_gui
  rqt_gui
  rqt_gui_py
  rospy
  std_msgs
  swarm_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mission_ui_node.py
  scripts/mission_ground_station.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
