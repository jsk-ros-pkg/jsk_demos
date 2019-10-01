cmake_minimum_required(VERSION 2.8.3)
project(jsk_2019_10_spatula)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
