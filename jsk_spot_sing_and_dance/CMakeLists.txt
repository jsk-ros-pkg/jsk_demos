cmake_minimum_required(VERSION 3.0.2)
project(jsk_spot_sing_and_dance)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
)

catkin_install_python(PROGRAMS node_scripts/sing_and_dance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)