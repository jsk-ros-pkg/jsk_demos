cmake_minimum_required(VERSION 2.8.3)
project(ar_sensor_view)

find_package(catkin REQUIRED COMPONENTS roseus)

catkin_package()

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

file(GLOB _install_files RELATIVE ${PROJECT_SOURCE_DIR} *.l *.launch)

install(FILES ${_install_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
install(FILES calc-polygon-to-coords.l
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PERMISSIONS WORLD_EXECUTE
  )
