cmake_minimum_required(VERSION 3.0.2)
project(px4_param_migrator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/param_migrator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
