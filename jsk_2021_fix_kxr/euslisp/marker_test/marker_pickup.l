#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(require "package://jsk_2021_fix_kxr/euslisp/marker_test/parameters.l")

(ros::roseus-add-msgs "std_msgs")

(ros::load-ros-manifest "apriltag_ros")

(ros::roseus "marker_test")

(dual_panda-init)

(objects (list *robot*))
(defvar *dual_arm_base-to-marker-coords*)
(defvar *rarm_end_effector-to-marker-coords*)

(defun grasp-marker (marker-name)
  (let ((base-to-marker-origin nil)
        (base-to-pre-grasp-pose nil)
        (base-to-grasp-pose nil)
	(stamp nil)
        (ret nil))
    (ros::ros-info "grasp a marker")
    (setq stamp (ros::time-now))
    (send *robot* :reset-pose)
    (send *robot* :rarm :move-end-pos #f(-50 0 0))
    (send *robot* :larm :move-end-pos #f(-50 0 0))
    (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
    (send *ri* :wait-interpolation)
    
    (when (send *tfl* :wait-for-transform "dual_arm_base" marker-name stamp 5)
      (setq base-to-marker-origin (send *tfl* :lookup-transform "dual_arm_base" marker-name stamp)))
    (setq base-to-grasp-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *marker-to-grasp-pose*))
    
    ;pre-grasp-pose
    (send *robot* :rarm :inverse-kinematics base-to-grasp-pose)
    (send *robot* :rarm :move-end-pos #f(-50 0 0))
    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    ;grasp-pose
    (send *robot* :rarm :inverse-kinematics base-to-grasp-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
    (send *ri* :wait-interpolation)

    (send *ri* :start-grasp :rarm)
    (unix:sleep 1)
    (send *ri* :stop-grasp :rarm)
    (unix:sleep 1)
    
    (unix:sleep 1)
    
    ;return to start-pose
    (send *robot* :reset-pose)
    (send *robot* :rarm :move-end-pos #f(-50 0 0))
    (send *robot* :larm :move-end-pos #f(-50 0 0))
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (ros::ros-info "finish")
    ))

