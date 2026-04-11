cmake_minimum_required(VERSION 2.8.3)
project(jsk_2025_10_semi)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

find_package(euscollada) # Just in case when euscollada not released. Avoid compile failing
if(NOT euscollada_FOUND)
  message(WARNING "euscollada is not found, so skip generating eus models.")
  return()
endif()

set(_xlerobot_urdf_dir /home/mech-user/venv_lerobot/XLeRobot/simulation/Maniskill/assets/xlerobot)
set(_collada2eus ${euscollada_PREFIX}/lib/euscollada/collada2eus)

foreach(ROBOT_TYPE xlerobot)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.l
    COMMAND echo "${_collada2eus} ${_xlerobot_urdf_dir}/${ROBOT_TYPE}.urdf ${ROBOT_TYPE}.l"
    COMMAND ${_collada2eus} ${_xlerobot_urdf_dir}/${ROBOT_TYPE}.urdf ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.yaml ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.l
    DEPENDS ${_xlerobot_urdf_dir}/${ROBOT_TYPE}.urdf ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.yaml ${_collada2eus})
  # add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}-simple.l
  #   COMMAND echo "${_collada2eus} --simple_geometry ${_xlerobot_urdf_dir}/${ROBOT_TYPE}.urdf ${ROBOT_TYPE}-simple.l"
  #   COMMAND ${_collada2eus} --simple_geometry ${_xlerobot_urdf_dir}/${ROBOT_TYPE}.urdf ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.yaml ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}-simple.l
  #   DEPENDS ${_xlerobot_urdf_dir}/${ROBOT_TYPE}.urdf ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.yaml ${_collada2eus})
  add_custom_target(compile_${ROBOT_TYPE} ALL DEPENDS ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}.l) # ${PROJECT_SOURCE_DIR}/${ROBOT_TYPE}-simple.l)
endforeach()


install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

