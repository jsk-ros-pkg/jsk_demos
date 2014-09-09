cmake_minimum_required(VERSION 2.8.3)
project(detect_cans_in_fridge_201202)
find_package(catkin REQUIRED COMPONENTS roseus jsk_perception jsk_pcl_ros jsk_pr2_startup snap_map_icp)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roseus jsk_perception jsk_pcl_ros jsk_pr2_startup snap_map_icp
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} python ${jsk_tools_PACKAGE_PATH}/bin/launchdoc-generator.py ${PROJECT_NAME} --output_dir=./build --nomakefile RESULT_VARIABLE _make_failed)
