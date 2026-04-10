cmake_minimum_required(VERSION 3.0.2)
project(database_talker)

find_package(catkin REQUIRED COMPONENTS catkin_virtualenv)

## This macro ensures modules and global scripts declared therein get installed
catkin_python_setup()

catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3
  CHECK_VENV FALSE
  USE_SYSTEM_PACKAGES FALSE  # Default TRUE
)

catkin_package(
)

catkin_install_python(PROGRAMS
  sample/include/store_image_description.py scripts/make_diary.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
