cmake_minimum_required(VERSION 2.8.3)
project(jsk_2013_05_pr2_tatu)

find_package(catkin REQUIRED)

catkin_package(
    DEPENDS #
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
