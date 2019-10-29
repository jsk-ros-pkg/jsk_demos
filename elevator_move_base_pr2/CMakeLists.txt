cmake_minimum_required(VERSION 2.8.3)
project(elevator_move_base_pr2)

find_package(catkin REQUIRED roscpp image_transport cv_bridge image_geometry tf geometry_msgs std_msgs message_generation)

add_custom_target(
  ${PROJECT_NAME}_install_trained_model
  ALL
  COMMAND ${PROJECT_SOURCE_DIR}/scripts/install_trained_model.py)

add_message_files(
  FILES Float32Stamped.msg)

generate_messages(
  DEPENDENCIES std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)

add_executable(color_point_detector src/color_point_detector.cpp)
target_link_libraries(color_point_detector ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test-color-point-detector.launch)
  add_rostest(test/test-panel-pose-detection.launch ROBOT:=sim)
  add_rostest(test/test-button-light.launch ROBOT:=sim)
  # 2 tests below are skipped now since they aren't actually used for testing purpose.
  add_rostest(test/test-modules-callpanel.launch ROBOT:=sim)
  add_rostest(test/test-modules-insidepanel.launch ROBOT:=sim)
endif()

install(DIRECTORY launch test scripts elevator_numbers
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(TARGETS color_point_detector
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
