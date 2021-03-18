cmake_minimum_required(VERSION 3.0.2)
project(jsk_spot_proximity_warning)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
)

catkin_install_python(PROGRAMS
  node_scripts/proximity_warning.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)