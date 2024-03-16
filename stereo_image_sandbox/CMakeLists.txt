cmake_minimum_required(VERSION 2.8.3)
project(stereo_image_sandbox)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  jsk_topic_tools
  nodelet
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)
message("OPENCV ${OpenCV_VERSION} FOUND")

catkin_package(
  CATKIN_DEPENDS
  camera_info_manager
  jsk_topic_tools
  nodelet
  sensor_msgs
  DEPENDS
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
)

# Build nodelet executables
# include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# ## Macro to add nodelets
# macro(stereo_image_sandbox_add_nodelet node_name nodelet_cppfile)
#   set(NODE_NAME ${node_name})
#   set(NODELET_NAME stereo_image_sandbox/${node_name})
#   configure_file(src/node/standalone_nodelet_exec.cpp.in ${node_name}.cpp @ONLY)
#   add_executable(${node_name}_exe ${node_name}.cpp)
#   SET_TARGET_PROPERTIES(${node_name}_exe PROPERTIES OUTPUT_NAME ${node_name})
#   target_link_libraries(${node_name}_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#   list(APPEND _stereo_image_sandbox_nodelet_cppfiles ${nodelet_cppfile})
#   list(APPEND _stereo_image_sandbox_nodelet_targets ${node_name}_exe)
# endmacro()


# # add_executable(dummy src/split_image.cpp)
# stereo_image_sandbox_add_nodelet("split_image" src/split_image.cpp)

# add_library(${PROJECT_NAME} SHARED
#   src/nodelet/nodelet.cpp
#   ${_stereo_image_sandbox_nodelet_cppfiles}
# )

# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
# install(TARGETS ${PROJECT_NAME}
#   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

# install(DIRECTORY plugins
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   USE_SOURCE_PERMISSIONS
# )
