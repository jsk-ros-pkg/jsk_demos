cmake_minimum_required(VERSION 2.8.3)
project(jsk_patrol)

find_package(catkin REQUIRED
  std_msgs
  message_generation
  jsk_perception
)

add_service_files(FILES
  PatrolMailNotify.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs
)
