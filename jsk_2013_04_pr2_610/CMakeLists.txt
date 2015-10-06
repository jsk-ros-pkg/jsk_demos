cmake_minimum_required(VERSION 2.8.3)
project(jsk_2013_04_pr2_610)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  jsk_demo_common
  jsk_perception
  task_compiler
  pddl_planner
  pddl_planner_viewer
  laser_filters_jsk_patch
  imagesift
  pr2eus
  roseus_smach
  roseus)

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
CATKIN_DEPENDS geometry_msgs message_runtime roseus jsk_demo_common jsk_perception pddl_planner pr2eus roseus_smach
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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test-demo.test)
endif()
