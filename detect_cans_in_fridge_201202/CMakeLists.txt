cmake_minimum_required(VERSION 2.8.3)
project(detect_cans_in_fridge_201202)

find_package(catkin REQUIRED COMPONENTS
  jsk_perception
  jsk_pcl_ros
  pr2_navigation_self_filter
  jsk_pr2_startup
  snap_map_icp
  jsk_demo_common
  pr2eus
  jsk_tools
  pr2_gripper_sensor_msgs
  pcl_msgs
  posedetection_msgs
  roseus
  )

catkin_package(
    DEPENDS
    CATKIN_DEPENDS jsk_perception jsk_pcl_ros pr2_navigation_self_filter jsk_pr2_startup snap_map_icp jsk_demo_common pr2eus jsk_tools pr2_gripper_sensor_msgs pcl_msgs posedetection_msgs roseus
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
