cmake_minimum_required(VERSION 2.8.3)
project(drc_com_common)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules message_generation std_msgs std_srvs sensor_msgs
  jsk_network_tools jsk_recognition_msgs visualization_msgs
  dynamic_reconfigure
  roseus)

add_message_files(FILES
  FC2OCSLarge.msg FC2OCSLargeReliable.msg FC2OCSSmall.msg OCS2FCSmall.msg
  FC2OCSBasicInfoSmall.msg
  VehicleOCS2FCSmall.msg
  VehicleFC2OCSSmall.msg
  HumanoidJointAngles.msg)
generate_messages(DEPENDENCIES sensor_msgs jsk_recognition_msgs visualization_msgs)
generate_dynamic_reconfigure_options(
  cfg/DRCParameters.cfg
  )

catkin_package()

install(DIRECTORY scripts launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
