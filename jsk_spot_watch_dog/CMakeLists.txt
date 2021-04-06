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

