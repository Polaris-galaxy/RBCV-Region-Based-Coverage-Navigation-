cmake_minimum_required(VERSION 3.0.2)
project(rbcv_bringup)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/regions_to_survey_zones.py
  scripts/rbcv_semantic_stack_cli.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

