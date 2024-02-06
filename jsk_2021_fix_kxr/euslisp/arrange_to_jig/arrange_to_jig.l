#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(require "package://jsk_2021_fix_kxr/euslisp/panda_commons/parameters.l")
(require "package://jsk_2021_fix_kxr/euslisp/panda_commons/utils.l")

(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::load-ros-manifest "franka_msgs")
(ros::load-ros-manifest "apriltag_ros")

(ros::roseus "grasp_and_insert_cable")

(dual_panda-init)

(objects (list *robot*))

(send *ri* :set-joint-pd-gain "larm_joint1" 1000 30)
(send *ri* :set-joint-pd-gain "larm_joint2" 1000 30)
(send *ri* :set-joint-pd-gain "larm_joint3" 1000 30)
(send *ri* :set-joint-pd-gain "larm_joint4" 1000 30)
(send *ri* :set-joint-pd-gain "larm_joint5" 500 15)
(send *ri* :set-joint-pd-gain "larm_joint6" 300 15)
(send *ri* :set-joint-pd-gain "larm_joint7" 100 10)
(send *ri* :set-joint-pd-gain "rarm_joint1" 1000 30)
(send *ri* :set-joint-pd-gain "rarm_joint2" 1000 30)
(send *ri* :set-joint-pd-gain "rarm_joint3" 1000 30)
(send *ri* :set-joint-pd-gain "rarm_joint4" 1000 30)
(send *ri* :set-joint-pd-gain "rarm_joint5" 500 15)
(send *ri* :set-joint-pd-gain "rarm_joint6" 300 15)
(send *ri* :set-joint-pd-gain "rarm_joint7" 100 10)

(setq search-body-result (search-body))
