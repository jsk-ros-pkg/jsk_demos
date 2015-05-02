#!/bin/bash

TF_TOPICS="/tf"
JOINT_STATES_TOPICS="/joint_states"
MULTISENSE_TOPICS=$(rostopic list | grep "multisense")
CAMERA_INFO_TOPICS=$(rostopic list | grep "camera_info")
IMU_TOPICS=$(rostopic list | grep "imu")
LASER_TOPICS=$(rostopic list | grep "scan")
ODOM_TOPICS=$(rostopic list | grep "odom")
VEHICLE_TOPICS=$(rostopic list | grep "drc_vehicle_xp900")

RECORD_TOPICS="${TF_TOPICS} ${JOINT_STATES_TOPICS} ${MULTISENSE_TOPICS} ${CAMERA_INFO_TOPICS} ${IMU_TOPICS} ${LASER_TOPICS} ${ODOM_TOPICS} ${VEHICLE_TOPICS}"
ROSBAG_ARGS="-o gazebo-drive-simulation"

rosbag record $ROSBAG_ARGS $RECORD_TOPICS

