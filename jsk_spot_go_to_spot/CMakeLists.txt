cmake_minimum_required(VERSION 3.0.2)
project(jsk_spot_go_to_spot)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
)

catkin_install_python(PROGRAMS node_scripts/go_to_spot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
