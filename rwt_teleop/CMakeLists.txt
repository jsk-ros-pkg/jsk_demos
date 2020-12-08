cmake_minimum_required(VERSION 2.8.3)
project(rwt_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  # std_msgs
  # geometry_msgs
  )

catkin_package(
  DEPENDS
  CATKIN_DEPENDS
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
