cmake_minimum_required(VERSION 3.0.2)
project(elevator_operation)

find_package(catkin REQUIRED genmsg actionlib_msgs)

add_message_files(DIRECTORY msg FILES DoorState.msg)

add_service_files(DIRECTORY srv FILES LookAtTarget.srv)

add_action_files(DIRECTORY action FILES MoveFloorWithElevator.action)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package()
