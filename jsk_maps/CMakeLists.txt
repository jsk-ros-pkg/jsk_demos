# only for catkin
cmake_minimum_required(VERSION 2.8.3)
project(jsk_maps)

find_package(catkin REQUIRED COMPONENTS
  roseus multi_map_server geometry_msgs)

if(roseus_SOURCE_DIR)
  set(roseus_PACKAGE_PATH ${roseus_SOURCE_DIR})
elseif(roseus_SOURCE_PREFIX)
  set(roseus_PACKAGE_PATH ${roseus_SOURCE_PREFIX})
else(roseus_SOURCE_PREFIX)
  set(roseus_PACKAGE_PATH ${roseus_PREFIX}/share/roseus)
endif()
  

if(EXISTS ${roseus_PREFIX}/bin/roseus)
  set(_roseus_exe ${roseus_PREFIX}/bin/roseus)
else(EXISTS ${roseus_PACKAGE_PATH}/bin/roseus)
  set(_roseus_exe ${roseus_PACKAGE_PATH}/bin/roseus)
endif()

if(EXISTS ${roseus_PREFIX}/scripts/generate-all-msg-srv.sh)
  set(_geneus_msg_exe ${roseus_PREFIX}/scripts/generate-all-msg-srv.sh)
else(EXISTS ${roseus_PACKAGE_PATH}/scripts/generate-all-msg-srv.sh)
  set(_geneus_msg_exe ${roseus_PACKAGE_PATH}/scripts/generate-all-msg-srv.sh)
endif()

catkin_package()

set(_generated_files
  ${PROJECT_SOURCE_DIR}/raw_maps/eng2-2f-0.05.yaml
  ${PROJECT_SOURCE_DIR}/raw_maps/eng2-7f-0.05.yaml
  ${PROJECT_SOURCE_DIR}/raw_maps/eng6-1f-0.05.yaml
  ${PROJECT_SOURCE_DIR}/raw_maps/eng6-3f-0.05.yaml
  ${PROJECT_SOURCE_DIR}/raw_maps/eng6-b1f-0.05.yaml
  ${PROJECT_SOURCE_DIR}/raw_maps/eng8-6f-0.05.yaml
  ${PROJECT_SOURCE_DIR}/raw_maps/eng2-1f-0.05.yaml
  ${PROJECT_SOURCE_DIR}/raw_maps/eng2-3f-0.05.yaml
  ${PROJECT_SOURCE_DIR}/raw_maps/eng2-8f-0.05.yaml 
  ${PROJECT_SOURCE_DIR}/raw_maps/eng6-2f-0.05.yaml
  ${PROJECT_SOURCE_DIR}/raw_maps/eng6-4f-0.05.yaml 
  ${PROJECT_SOURCE_DIR}/raw_maps/eng8-1f-0.05.yaml
  ${PROJECT_SOURCE_DIR}/raw_maps/ eng8-b1f-0.05.yaml
  ${PROJECT_SOURCE_DIR}/launch/start_map_eng2_main.launch
  ${PROJECT_SOURCE_DIR}/launch/start_map_eng6_main.launch
  ${PROJECT_SOURCE_DIR}/launch/start_map_eng8_main.launch
  )

add_custom_command(
  OUTPUT ${_generated_files}
  COMMAND ${_roseus_exe} ${PROJECT_SOURCE_DIR}/src/dump-map-info.l
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

add_custom_target(jsk_maps_install ALL DEPENDS ${_generated_files})

install(DIRECTORY launch raw_maps src tools
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
