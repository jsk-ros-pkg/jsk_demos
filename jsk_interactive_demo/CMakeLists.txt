cmake_minimum_required(VERSION 3.0.2)
project(jsk_interactive_demo)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
  # std_msgs
  # message_runtime
)

catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3
  CHECK_VENV FALSE
)

catkin_package(
  CATKIN_DEPENDS 
)

# add_message_files(
#   FILES
#   task_state.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

# catkin_install_python(PROGRAMS
#   scripts/hoge.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
