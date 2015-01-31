cmake_minimum_required(VERSION 2.8.3)
project(jsk_2013_04_pr2_610)

find_package(catkin REQUIRED COMPONENTS roseus)

add_message_files(
  FILES
  geometry_quaternions.msg
  geometry_vectors.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jsk_2013_04_pr2_610
#  CATKIN_DEPENDS
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(
  DIRECTORY euslisp pddl
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
