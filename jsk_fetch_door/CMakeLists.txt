cmake_minimum_required(VERSION 2.8.3)
project(jsk_fetch_door)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
