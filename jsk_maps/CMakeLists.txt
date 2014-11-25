# only for catkin
cmake_minimum_required(VERSION 2.8.3)
project(jsk_maps)

find_package(catkin REQUIRED COMPONENTS
  roseus multi_map_server)

set(_roseus_exe ${roseus_PREFIX}/bin/roseus)

catkin_package()

add_custom_command(
  OUTPUT installed
  COMMAND ${_roseus_exe} ${PROJECT_SOURCE_DIR}/src/dump-map-info.l
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

add_custom_target(jsk_maps_install ALL DEPENDS installed)

install(DIRECTORY launch raw_maps src tools
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
