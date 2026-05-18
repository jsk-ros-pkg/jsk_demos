cmake_minimum_required(VERSION 3.0.2)
project(detect_circle)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  # message_generation
)

# add_message_files(
#   FILES
# PointStampedArray.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

# catkin_package(
#   CATKIN_DEPENDS cv_bridge roscpp sensor_msgs std_msgs geometry_msgs message_runtime
# )

catkin_package(
  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/detect_circle.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)