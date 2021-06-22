cmake_minimum_required(VERSION 2.8.3)
project(jsk_2020_10_semi)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

# create uml pictures
find_program(PLANTUML_EXE plantuml)
message(STATUS "check ${PLANTUML_EXE} found")
if(PLANTUML_EXE)
  file(GLOB UML_FILES RELATIVE ${PROJECT_SOURCE_DIR}/uml/ ${PROJECT_SOURCE_DIR}/uml/*.uml )
  foreach(UML_FILE ${UML_FILES})
    get_filename_component(UML_OUTPUT_FILE ${UML_FILE} NAME_WE)
    add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/uml/${UML_OUTPUT_FILE}.png
      COMMAND ${PLANTUML_EXE} ${PROJECT_SOURCE_DIR}/uml/${UML_FILE}
      DEPENDS ${PROJECT_SOURCE_DIR}/uml/${UML_FILE})
    add_custom_target(compile_${UML_FILE} ALL DEPENDS ${PROJECT_SOURCE_DIR}/uml/${UML_OUTPUT_FILE}.png)
  endforeach()
endif()
