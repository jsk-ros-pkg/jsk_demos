#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(require "package://jsk_2021_fix_kxr/euslisp/panda_commons/parameters.l")
(require "package://jsk_2021_fix_kxr/euslisp/panda_commons/utils.l")
(require "package://jsk_2021_fix_kxr/euslisp/main/panda-repair-rarm-elbow-p-servo.l")
(require "package://jsk_2021_fix_kxr/euslisp/main/panda-repair-larm-elbow-p-servo.l")

(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::load-ros-manifest "franka_msgs")
(ros::load-ros-manifest "apriltag_ros")
(ros::load-ros-manifest "opencv_apps")
(ros::load-ros-manifest "jsk_2021_fix_kxr")

(ros::roseus "panda_robot")

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
(send *robot* :angle-vector (send *ri* :state :potentio-vector))

(ros::ros-info "Start repairing")

(send *robot* :reset-coop-pose)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(while t
  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "send-servo-info")
  (setq res (ros::service-call "repair_reaction" req t))
  (setq joint_states (send res :joint_state))
  (setq timeout-servo-list (make-timeout-servo-list joint_states))
  (if timeout-servo-list
      (progn
	(setq target (decide-target-servo timeout-servo-list))
	(if (string-equal target "rarm-elbow-p")
	    (progn
	      (ros::ros-info "target is rarm-elbow-p-servo")
	      (panda-repair-rarm-elbow-p-servo)))
	(if (string-equal target "larm-elbow-p")
	    (progn
	      (ros::ros-info "target is larm-elbow-p-servo")
	      (panda-repair-larm-elbow-p-servo))))
    (return-from nil)))

(ros::ros-info "Finished repairing")
