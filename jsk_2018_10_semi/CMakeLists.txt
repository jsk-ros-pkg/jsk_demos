cmake_minimum_required(VERSION 2.8.3)
project(jsk_2018_10_semi)

find_package(catkin REQUIRED COMPONENTS
  fetcheus
  jsk_maps
  naoeus
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark euslisp files for installation
install(DIRECTORY euslisp launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_jsk_2018_10_semi.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
