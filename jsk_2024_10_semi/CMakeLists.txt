cmake_minimum_required(VERSION 2.8.3)
project(jsk_2024_10_semi)

find_package(catkin REQUIRED COMPONENTS catkin_virtualenv)

catkin_package()

# Generate the virtualenv
catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3.10
  USE_SYSTEM_PACKAGES FALSE  # Default TRUE

  # Provide extra arguments to the underlying pip invocation
  #EXTRA_PIP_ARGS  -vvv
  )

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(
  PROGRAMS scripts/conversation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
