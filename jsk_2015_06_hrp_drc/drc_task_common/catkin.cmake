cmake_minimum_required(VERSION 2.8.3)
project(drc_task_common)

find_package(catkin REQUIRED COMPONENTS cmake_modules message_generation std_msgs std_srvs geometry_msgs roscpp rospy sensor_msgs visualization_msgs message_filters message_generation jsk_pcl_ros interactive_markers tf pcl_conversions jsk_topic_tools rviz eigen_conversions)
catkin_python_setup()

add_message_files(DIRECTORY msg FILES StringMultiArray.msg)
add_message_files(DIRECTORY msg FILES InteractiveMarkerArray.msg)
add_service_files(DIRECTORY srv FILES RvizMenuCall.srv RvizMenuSelect.srv EusCommand.srv StringRequest.srv ICPService.srv)

generate_messages(DEPENDENCIES ${PCL_MSGS} std_msgs std_srvs visualization_msgs sensor_msgs geometry_msgs jsk_pcl_ros jsk_interactive_marker)

catkin_package(
CATKIN_DEPENDS message_runtime
)
include_directories(
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
install(DIRECTORY scripts launcha
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

add_executable(manipulation_data_server src/drc_task_common/manipulation_data_server.cpp)
target_link_libraries(manipulation_data_server
   ${catkin_LIBRARIES}
   yaml-cpp
)
