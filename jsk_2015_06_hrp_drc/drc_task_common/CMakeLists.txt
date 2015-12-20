cmake_minimum_required(VERSION 2.8.3)
project(drc_task_common)

find_package(catkin REQUIRED COMPONENTS cmake_modules message_generation std_msgs std_srvs geometry_msgs roscpp rospy sensor_msgs visualization_msgs message_filters message_generation jsk_pcl_ros interactive_markers pcl_conversions jsk_topic_tools rviz eigen_conversions dynamic_tf_publisher jsk_interactive_marker jsk_recognition_msgs move_base_msgs rosgraph_msgs topic_tools jsk_topic_tools jsk_ik_server pcl_msgs jsk_footstep_msgs drc_com_common jsk_perception jsk_calibration resized_image_transport smach_msgs pcl_ros nav_msgs tf dynamic_reconfigure roseus)

catkin_python_setup()

add_message_files(DIRECTORY msg FILES StringMultiArray.msg)
add_message_files(DIRECTORY msg FILES InteractiveMarkerArray.msg)
add_message_files(DIRECTORY msg FILES TMarkerInfo.msg)
add_message_files(DIRECTORY msg FILES Float32ArrayStamped.msg)
add_message_files(DIRECTORY msg FILES Int8Float64.msg)
add_message_files(DIRECTORY msg FILES JointStateArray.msg)
add_service_files(DIRECTORY srv FILES RvizMenuCall.srv RvizMenuSelect.srv EusCommand.srv StringRequest.srv ICPService.srv GetIKArm.srv GetIKArmPose.srv GoPosCommand.srv Uint8Request.srv AngleVectorCommand.srv ReachUntilTouchCommand.srv SetValue.srv GetBoolState.srv)

# Generate msg file from DRCParametersConfig file
execute_process(
  COMMAND ${PROJECT_SOURCE_DIR}/scripts/gen_reconfigure_msg.py ${PROJECT_SOURCE_DIR}/msg/DRCParametersMessage.msg
  ${PROJECT_SOURCE_DIR}/msg/DRCParametersMessage.msg)

generate_dynamic_reconfigure_options(
  config/vehicle/LocalPlannerParams.cfg
  config/vehicle/ObstacleDetectionParams.cfg
  cfg/ValveRejector.cfg
  # config/vehicle/PassthroughCarBody.cfg
  # config/vehicle/ObstacleIndicatorParams.cfg
  cfg/StandingDrillDetector.cfg
  cfg/FilterBBoxPosition.cfg
  cfg/BControlDummyServer.cfg
)

add_message_files(DIRECTORY msg FILES DRCParametersMessage.msg)

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

add_executable(manipulation_data_server src/drc_task_common/manipulation_data_server.cpp src/drc_task_common/manipulation_data_helpers.cpp)
target_link_libraries(manipulation_data_server
  ${catkin_LIBRARIES}
  yaml-cpp
  )
add_dependencies(manipulation_data_server ${PROJECT_NAME}_gencpp)

add_executable(standing_drill_detector src/drc_task_common/standing_drill_detector.cpp)
target_link_libraries(standing_drill_detector
  ${catkin_LIBRARIES}
  yaml-cpp
  )
add_dependencies(standing_drill_detector ${PROJECT_NAME}_gencpp)

add_executable(manipulation_data_visualizer src/drc_task_common/manipulation_data_visualizer.cpp src/drc_task_common/manipulation_data_helpers.cpp)
target_link_libraries(manipulation_data_visualizer
   ${catkin_LIBRARIES}
   yaml-cpp
)
add_dependencies(manipulation_data_visualizer ${PROJECT_NAME}_gencpp)


add_executable(kdtree_obstacle src/drc_task_common/kdtree_obstacle_detection.cpp)
add_dependencies(kdtree_obstacle ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries(kdtree_obstacle ${catkin_LIBRARIES})

add_executable(simplified_local_planner src/drc_task_common/simplified_local_planner.cpp)
add_dependencies(simplified_local_planner ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries(simplified_local_planner ${catkin_LIBRARIES})

add_executable(passthrough_car_body src/drc_task_common/passthrough_car_body.cpp)
add_dependencies(passthrough_car_body ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries(passthrough_car_body ${catkin_LIBRARIES})

add_executable(obstacle_indicator src/drc_task_common/obstacle_indicator.cpp)
add_dependencies(obstacle_indicator ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries(obstacle_indicator ${catkin_LIBRARIES})

execute_process(
  COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/pcds)
catkin_download_test_data(drill_pcd
  http://www.jsk.t.u-tokyo.ac.jp/~ueda/dataset/2015/02/drill.pcd
  DESTINATION ${PROJECT_SOURCE_DIR}/pcds)
# catkin_download_test_data(drill_full_pcd
#   http://www.jsk.t.u-tokyo.ac.jp/~ueda/dataset/2015/02/drill_full.pcd
#   DESTINATION ${PROJECT_SOURCE_DIR}/pcds)
if(NOT ($ENV{TRAVIS_JOB_ID}))
  execute_process(
    COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/models)
  catkin_download_test_data(gun_drill_dae
    http://www.jsk.t.u-tokyo.ac.jp/~ohara/dataset/2015/03/gun_drill.dae
    DESTINATION ${PROJECT_SOURCE_DIR}/models)
  catkin_download_test_data(gun_drill_jpg
    http://www.jsk.t.u-tokyo.ac.jp/~ohara/dataset/2015/03/gun_drill_color.jpg
    DESTINATION ${PROJECT_SOURCE_DIR}/models)
  catkin_download_test_data(gun_drill_model
    http://www.jsk.t.u-tokyo.ac.jp/~ohara/dataset/2015/03/gun-drill.l
    DESTINATION ${PROJECT_SOURCE_DIR}/models)
  catkin_download_test_data(takenoko_drill_dae
    http://www.jsk.t.u-tokyo.ac.jp/~ohara/dataset/2015/03/takenoko_drill.dae
    DESTINATION ${PROJECT_SOURCE_DIR}/models)
  catkin_download_test_data(takenoko_drill_jpg
    http://www.jsk.t.u-tokyo.ac.jp/~ohara/dataset/2015/03/dewalt_takenoko_new-best-exported01.jpg
    DESTINATION ${PROJECT_SOURCE_DIR}/models)
  catkin_download_test_data(takenoko_drill_model
    http://www.jsk.t.u-tokyo.ac.jp/~ohara/dataset/2015/03/takenoko-drill.l
    DESTINATION ${PROJECT_SOURCE_DIR}/models)
  add_custom_target(all_drc_task_common_downloads ALL DEPENDS
    drill_pcd # todo old now
    # drill_full_pcd
    gun_drill_dae gun_drill_jpg gun_drill_model takenoko_drill_dae takenoko_drill_jpg takenoko_drill_model)
endif(NOT ($ENV{TRAVIS_JOB_ID}))

install(TARGETS
  standing_drill_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY scripts launch icons
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
