cmake_minimum_required(VERSION 3.0.2)
project(jsk_spot_go_back_home)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY euslisp launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
