cmake_minimum_required(VERSION 2.8.3)
project(drc_com_common)

find_package(catkin REQUIRED COMPONENTS cmake_modules message_generation std_msgs std_srvs sensor_msgs)

add_message_files(FILES 
  FC2OCSLarge.msg FC2OCSSmall.msg OCS2FCSmall.msg)
generate_messages(DEPENDENCIES sensor_msgs)


catkin_package()

install(DIRECTORY scripts launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
