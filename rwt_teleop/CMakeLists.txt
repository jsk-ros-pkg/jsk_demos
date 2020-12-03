cmake_minimum_required(VERSION 2.8.3)
project(rwt_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  # std_msgs
  # geometry_msgs
  )

catkin_package(
  DEPENDS
  CATKIN_DEPENDS
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# gazebo plugins
find_package(gazebo REQUIRED)
#find_package(ignition-transport4 REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# # copied from contain_plugin tutorials
# # http://gazebosim.org/tutorials?tut=contain_plugin&cat=plugins
# add_library(TurnOnLightPlugin SHARED src/TurnOnLightPlugin.cpp)
# target_link_libraries(TurnOnLightPlugin
#   ${GAZEBO_LIBRARIES}
#   ignition-transport4::ignition-transport4
# )

# # contain_plugin is released with gazebo 9.1
# # https://github.com/osrf/gazebo/tree/gazebo9_9.1.0/plugins
# add_library(ContainPlugin SHARED src/ContainPlugin.cc)
# target_link_libraries(ContainPlugin
#   ${GAZEBO_LIBRARIES}
#   ignition-transport4::ignition-transport4
# )

