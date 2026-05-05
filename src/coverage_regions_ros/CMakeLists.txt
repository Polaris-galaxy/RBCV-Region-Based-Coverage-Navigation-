cmake_minimum_required(VERSION 3.0.2)
# TODO: 与 package.xml <name> 同步改名
project(coverage_regions_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/partition_labels_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

