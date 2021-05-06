cmake_minimum_required(VERSION 3.0.2)
project(spot_person_leader)

add_compile_options(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib actionlib_msgs rospy)

add_action_files(
    DIRECTORY action
    FILES LeadPerson.action
)

add_service_files(
    DIRECTORY srv
    FILES GetStairRanges.srv
)

add_message_files(
    DIRECTORY msg
    FILES StairRange.msg
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
)

include_directories(
)
