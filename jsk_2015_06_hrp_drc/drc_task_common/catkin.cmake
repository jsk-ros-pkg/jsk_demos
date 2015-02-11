cmake_minimum_required(VERSION 2.8.3)
project(drc_task_common)

find_package(catkin REQUIRED COMPONENTS cmake_modules message_generation std_msgs std_srvs geometry_msgs roscpp rospy sensor_msgs visualization_msgs message_filters message_generation jsk_pcl_ros interactive_markers pcl_conversions jsk_topic_tools rviz eigen_conversions dynamic_tf_publisher jsk_interactive_marker jsk_recognition_msgs move_base_msgs rosgraph_msgs roseus)
catkin_python_setup()

add_message_files(DIRECTORY msg FILES StringMultiArray.msg)
add_message_files(DIRECTORY msg FILES InteractiveMarkerArray.msg)
add_message_files(DIRECTORY msg FILES TMarkerInfo.msg)
add_service_files(DIRECTORY srv FILES RvizMenuCall.srv RvizMenuSelect.srv EusCommand.srv StringRequest.srv ICPService.srv GetIKArm.srv GetIKArmPose.srv)

generate_messages(DEPENDENCIES ${PCL_MSGS} std_msgs std_srvs visualization_msgs sensor_msgs geometry_msgs jsk_pcl_ros jsk_interactive_marker jsk_recognition_msgs move_base_msgs)


catkin_package(
  CATKIN_DEPENDS message_runtime INCLUDE_DIRS
)

find_package(PkgConfig)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)
if(${yaml_cpp_VERSION} VERSION_LESS "0.5.0")
## indigo yaml-cpp : 0.5.0 /  hydro yaml-cpp : 0.3.0
  add_definitions("-DUSE_OLD_YAML")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS -g)

qt4_wrap_ui(UIC_FILES
  config/drc_teleop_interface.ui
  )
include_directories(${CMAKE_CURRENT_BINARY_DIR})

qt4_wrap_cpp(MOC_FILES
  src/drc_task_common/drc_teleop_interface.h
)
set(SOURCE_FILES
  src/drc_task_common/drc_teleop_interface.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES} ${UIC_FILES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons
)
install(DIRECTORY scripts launch
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

add_executable(manipulation_data_server src/drc_task_common/manipulation_data_server.cpp src/drc_task_common/manipulation_data_helpers.cpp)
target_link_libraries(manipulation_data_server
   ${catkin_LIBRARIES}
   yaml-cpp
)
add_dependencies(manipulation_data_server ${PROJECT_NAME}_gencpp)
add_executable(manipulation_data_visualizer src/drc_task_common/manipulation_data_visualizer.cpp src/drc_task_common/manipulation_data_helpers.cpp)
target_link_libraries(manipulation_data_visualizer
   ${catkin_LIBRARIES}
   yaml-cpp
)
add_dependencies(manipulation_data_visualizer ${PROJECT_NAME}_gencpp)
