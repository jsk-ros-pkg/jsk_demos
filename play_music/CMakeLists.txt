cmake_minimum_required(VERSION 3.0.2)
project(play_music)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
)

catkin_install_python(PROGRAMS node_scripts/play_music.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
