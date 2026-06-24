cmake_minimum_required(VERSION 3.0.2)
project(spot_person_follower)

add_compile_options(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib_msgs geometry_msgs rospy)

catkin_package(
)

include_directories(
)
