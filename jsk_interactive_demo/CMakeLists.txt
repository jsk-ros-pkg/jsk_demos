cmake_minimum_required(VERSION 3.0.2)
project(jsk_interactive_demo)

find_package(catkin REQUIRED COMPONENTS catkin_virtualenv)

catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3
  CHECK_VENV FALSE
)

catkin_package(
)

catkin_install_python(PROGRAMS
  scripts/hoge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
