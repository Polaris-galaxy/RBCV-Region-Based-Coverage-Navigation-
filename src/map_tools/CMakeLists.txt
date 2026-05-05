cmake_minimum_required(VERSION 3.0.2)
project(map_tools)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  std_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/publish_static_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY maps launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
