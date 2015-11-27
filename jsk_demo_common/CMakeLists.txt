cmake_minimum_required(VERSION 2.8.3)
project(jsk_demo_common)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  jsk_hark_msgs
  jsk_maps
  jsk_perception
  pddl_planner
  pr2eus
  pr2_gripper_sensor_msgs
  roseus
  roseus_smach
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES DemoCommon.action
)



## Generate added messages and services with any dependencies listed here       
add_message_files(
  FILES
  EventHeader.msg
  FunctionEvent.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs actionlib_msgs std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jsk_demo_common
  CATKIN_DEPENDS control_msgs jsk_hark_msgs jsk_maps jsk_perception pddl_planner pr2eus roseus roseus_smach message_runtime
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

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS)
