cmake_minimum_required(VERSION 3.0.2)
project(jsk_spot_take_envelop_to_mech_office)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
)

catkin_install_python(PROGRAMS node_scripts/take_envelop_to_mech_office.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
