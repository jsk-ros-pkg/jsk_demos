# only for catkin
cmake_minimum_required(VERSION 2.8.3)
project(jsk_maps)

find_package(catkin REQUIRED COMPONENTS
  roseus multi_map_server geometry_msgs)

catkin_package()

install(DIRECTORY launch raw_maps src tools
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
