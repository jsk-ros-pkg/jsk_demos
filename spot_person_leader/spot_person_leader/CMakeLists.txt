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
    FILES ResetCurrentNode.srv
)

generate_messages(DEPENDENCIES actionlib_msgs)

add_custom_target(${PROJECT_NAME}_download_autowalk_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/download_autowalk_data.py)

catkin_package(
)

include_directories(
)
