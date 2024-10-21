cmake_minimum_required(VERSION 2.8.3)
project(semi2024)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  jsk_topic_tools
  nodelet
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)
message("OPENCV ${OpenCV_VERSION} FOUND")

catkin_package(
  CATKIN_DEPENDS
  camera_info_manager
  jsk_topic_tools
  nodelet
  sensor_msgs
  DEPENDS
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
)
