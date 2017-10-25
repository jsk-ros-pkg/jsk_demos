# only for catkin
cmake_minimum_required(VERSION 2.8.3)
project(jsk_maps)

find_package(catkin REQUIRED)
find_package(euslisp REQUIRED)

if(${CMAKE_SYSTEM_NAME} MATCHES Linux)
  execute_process(COMMAND gcc -dumpmachine OUTPUT_VARIABLE GCC_MACHINE OUTPUT_STRIP_TRAILING_WHITESPACE)
  message("-- Set GCC_MACHINE to ${GCC_MACHINE}")
  if(${GCC_MACHINE} MATCHES x86_64-linux-gnu)
    set(ARCHDIR Linux64)
  elseif(${GCC_MACHINE} MATCHES i686-linux-gnu)
    set(ARCHDIR Linux)
  elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
      ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* )
    set(ARCHDIR Linux64)
  elseif(${GCC_MACHINE} MATCHES arm-linux-*)
    set(ARCHDIR LinuxARM)
  else()
    set(ARCHDIR Linux)
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES Darwin)
  set(ARCHDIR Darwin)
elseif(${CMAKE_SYSTEM_NAME} MATCHES Cygwin)
  set(ARCHDIR Cygwin)
else()
  set(ARCHDIR Generic)
endif()

if (EXISTS ${euslisp_SOURCE_DIR}/jskeus/eus/${ARCHDIR}/bin/irteusgl)
  set(_eus_exe ${euslisp_SOURCE_DIR}/jskeus/eus/${ARCHDIR}/bin/irteusgl)
elseif (EXISTS ${euslisp_SOURCE_PREFIX}/jskeus/eus/${ARCHDIR}/bin/irteusgl)
  set(_eus_exe ${euslisp_SOURCE_PREFIX}/jskeus/eus/${ARCHDIR}/bin/irteusgl)
elseif (EXISTS ${euslisp_PREFIX}/share/euslisp/jskeus/eus/${ARCHDIR}/bin/irteusgl)
  set(_eus_exe ${euslisp_PREFIX}/share/euslisp/jskeus/eus/${ARCHDIR}/bin/irteusgl)
else (EXISTS ${euslisp_SOURCE_DIR}/jskeus/eus/${ARCHDIR}/bin/irteusgl)
  message(FATAL_ERROR "cannot find irteusgl")
endif(EXISTS ${euslisp_SOURCE_DIR}/jskeus/eus/${ARCHDIR}/bin/irteusgl)

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
  COMMAND DISPLAY= ${_eus_exe} src/catkin-build-util.l
  DEPENDS ${PROJECT_SOURCE_DIR}/src/catkin-build-util.l ${PROJECT_SOURCE_DIR}/src/dump-map-info.l
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

add_custom_target(jsk_maps_install ALL DEPENDS ${_generated_files})

install(DIRECTORY launch raw_maps src tools
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(jsk_tools REQUIRED)
  if (${jsk_tools_VERSION} VERSION_LESS "2.0.9")
    message(WARNING "To test this package, 'jsk_tools>=2.0.9' is required. Skipping...")
  else()
    add_rostest(test/jsk-maps.test)
  endif()
endif()
