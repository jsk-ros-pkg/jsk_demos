cmake_minimum_required(VERSION 2.8.3)
project(jsk_2011_07_pr2_semantic)
find_package(catkin REQUIRED)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

#############
## Install ##
#############

install(DIRECTORY data euslisp launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
