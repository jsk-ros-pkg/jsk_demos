cmake_minimum_required(VERSION 2.8.3)
project(detect_cans_in_fridge_201202)

find_package(catkin REQUIRED roseus)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS #
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

#############
## Install ##
#############

install(DIRECTORY data scripts config euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES detect_cans_in_fridge.rviz object_models1.yaml object_models_new.yaml self_filter.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
