cmake_minimum_required(VERSION 2.8.3)
project(jsk_2017_10_semi)

find_package(catkin REQUIRED COMPONENTS)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package()

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
