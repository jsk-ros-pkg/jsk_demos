#!/bin/bash

(
    echo "start rhp4 record"
    rosbag record -o ~/ishiguro/rhp4_teleop \
/act_capture_point \
/act_contact_states \
/clock \
/camera/color/image_raw/compressed \
/camera/color/image_raw/camera_info \
/imu \
/joint_states \
/lfsensor \
/lfsensor_cop \
/lhsensor \
/lhsensor_cop \
/master_com_pose \
/master_head_pose \
/master_larm_pose \
/master_larm_wrench \
/master_lhand_pose \
/master_lleg_pose \
/master_lleg_wrench \
/master_rarm_pose \
/master_rarm_wrench \
/master_rhand_pose \
/master_rleg_pose \
/master_rleg_wrench \
/master_lfloor_pose \
/master_rfloor_pose \
/odom \
/off_lfsensor \
/off_lhsensor \
/off_rfsensor \
/off_rhsensor \
/pc_monitor/tablis/eth1/receive \
/pc_monitor/tablis/eth1/transmit \
/ref_capture_point \
/ref_contact_states \
/ref_lfsensor \
/ref_lhsensor \
/ref_rfsensor \
/ref_rhsensor \
/rfsensor \
/rfsensor_cop \
/rhsensor \
/rhsensor_cop \
/robotsound \
/shm_servo_state \
/slave_larm_wrench \
/slave_lleg_wrench \
/slave_rarm_wrench \
/slave_rleg_wrench \
/slave_com_pose \
/slave_head_pose \
/slave_larm_pose \
/slave_lleg_pose \
/slave_rarm_pose \
/slave_rleg_pose \
/slave_lfloor_pose \
/slave_rfloor_pose \
/tf \
/tf_static \
/zmp

)
