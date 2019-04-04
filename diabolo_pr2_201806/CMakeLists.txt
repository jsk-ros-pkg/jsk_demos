cmake_minimum_required(VERSION 2.8.3)
project(diabolo_pr2_201806)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  pcl_conversions
  pcl_ros
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jsk_2017_10_semi
#  CATKIN_DEPENDS fetcheus jsk_maps pr2eus
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(calc_idle_diabolo_state src/calc_idle_diabolo_state.cpp)
target_link_libraries(calc_idle_diabolo_state ${catkin_LIBRARIES})

install(DIRECTORY euslisp gif launch scripts src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
