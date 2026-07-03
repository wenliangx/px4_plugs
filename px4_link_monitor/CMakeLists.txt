cmake_minimum_required(VERSION 3.0.2)
project(px4_link_monitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  diagnostic_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/link_monitor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
