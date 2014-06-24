cmake_minimum_required(VERSION 2.8.3)
project(jsk_2014_06_pr2_drcbox)

find_package(catkin REQUIRED COMPONENTS pr2eus)

catkin_package(CATKIN_DEPENDS pr2eus)

install(DIRECTORY euslisp launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
