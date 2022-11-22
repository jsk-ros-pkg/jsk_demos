cmake_minimum_required(VERSION 2.8.3)
project(kitchen_demo_bt)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  behaviortree_ros
  roseus_bt
  std_msgs
  actionlib_msgs
)
find_package(fmt)

add_service_files(
  FILES

)

add_action_files(
  FILES
  AutoDock.action
  Finish.action
  GetLightOn.action
  Init.action
  InspectSpot.action
  MoveToSpot.action
  ReportAutoDockFailure.action
  ReportFailure.action
  ReportLightOn.action
  ReportStartGoToKitchen.action
  RoomLightOff.action
  RoomLightOn.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  actionlib_msgs
  DEPENDS fmt
)


include_directories(${catkin_INCLUDE_DIRS})
add_subdirectory(${roseus_bt_SOURCE_PREFIX}/include/rosbridgecpp rosbridgecpp)

add_executable(kitchen_demo src/kitchen_demo.cpp)
add_dependencies(kitchen_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kitchen_demo ${catkin_LIBRARIES} rosbridgecpp fmt::fmt)


