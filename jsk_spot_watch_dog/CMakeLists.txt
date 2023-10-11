cmake_minimum_required(VERSION 3.0.2)
project(jsk_spot_watch_dog)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/watch_dog.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
