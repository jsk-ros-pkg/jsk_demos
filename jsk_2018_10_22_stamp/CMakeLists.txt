cmake_minimum_required(VERSION 2.8.3)
project(jsk_2018_10_22_stamp)

find_package(catkin REQUIRED COMPONENTS
  std_msgs geometry_msgs jsk_recognition_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  LabelWithGeometry.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs jsk_recognition_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs jsk_recognition_msgs
)

###########
## Build ##
###########

#############
## Install ##
#############

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
#   find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
#   file(GLOB LAUNCH_FILES launch/*.launch)
#   foreach(LAUNCH_FILE ${LAUNCH_FILES})
#     message(STATUS "Testing ${LAUNCH_FILE}")
#     roslaunch_add_file_check(${LAUNCH_FILE})
#   endforeach()
# endif()
