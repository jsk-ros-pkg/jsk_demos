cmake_minimum_required(VERSION 2.8.3)
project(interactive_behavior_201409)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  people_msgs
  pr2eus
  roseus
  std_msgs
)

# catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Attention.msg
  DialogResponse.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  EnqueueTask.srv
  InterruptTask.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

# TODO

#############
## Install ##
#############

install(DIRECTORY launch scripts euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
