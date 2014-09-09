cmake_minimum_required(VERSION 2.8.3)
project(jsk_maps)
find_package(catkin REQUIRED COMPONENTS roseus multi_map_server)

catkin_package(
    DEPENDS imagemagick
    CATKIN_DEPENDS roseus multi_map_server
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
