cmake_minimum_required(VERSION 3.0.2)
project(lead_pos_server)

find_package(catkin REQUIRED actionlib actionlib_msgs geometry_msgs)

add_action_files(
DIRECTORY action  
FILES
  LeadPos.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
)
