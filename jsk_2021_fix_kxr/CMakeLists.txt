cmake_minimum_required(VERSION 2.8.3)
project(jsk_2021_fix_kxr)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
 )

add_message_files(
  FILES
  Repair.msg
  )

add_service_files(
  FILES
  RepairInfo.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_lecture
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
