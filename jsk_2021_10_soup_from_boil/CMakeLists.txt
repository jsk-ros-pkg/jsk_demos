cmake_minimum_required(VERSION 2.8.3)
project(jsk_2021_10_soup_from_boil)

find_package(catkin REQUIRED
  )

catkin_package(
)

include_directories(
)

include_directories(include ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/soup-from-boil.test ARGS test_ik:=true)
  add_rostest(test/soup-from-boil.test ARGS test_ik:=false)
  add_rostest(test/take-tools-off-shelf.test ARGS test_ik:=true)
  add_rostest(test/take-tools-off-shelf.test ARGS test_ik:=false)
endif()
