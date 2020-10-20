cmake_minimum_required(VERSION 2.8.3)
project(robot_keypose_detection)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# download and install trained data
add_custom_target(${PROJECT_NAME}_install_trained_data ALL COMMAND ${PROJECT_SOURCE_DIR}/scripts/install_trained_data.py)
