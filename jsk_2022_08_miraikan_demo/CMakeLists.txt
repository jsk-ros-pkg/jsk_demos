cmake_minimum_required(VERSION 2.8.3)
project(miraikan_demo)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_service_files(DIRECTORY srv FILES
  Mode.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package()

install(DIRECTORY euslisp launch scripts srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

include_directories()
