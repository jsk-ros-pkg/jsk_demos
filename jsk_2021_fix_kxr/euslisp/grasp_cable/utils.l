#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(require "package://jsk_2021_fix_kxr/euslisp/grasp_cable/parameters.l")
(ros::roseus-add-msgs "std_msgs")

(ros::load-ros-manifest "apriltag_ros")

(ros::roseus "grasp_cable")

(dual_panda-init)

(objects (list *robot*))

(defvar *marker-000*)
(defvar *marker-001*)
(defvar *marker-002*)

(send *robot* :reset-pose)
(send *robot* :rarm :move-end-pos #f(-500 0 0))
(send *robot* :larm :move-end-pos #f(-500 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(350.616 -340 1100) :rpy (float-vector 0.0 1.0 2.96))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(defun rarm-grasp-cable (servo-name)
  (let ((base-to-marker-origin nil)
	(base-to-pre-prasp-pose nil)
	(base-to-grasp-pose nil)
	(stamp nil)
	(ret nil))
    (ros::ros-info "grasp cable")
    (setq stamp (ros::time-now))
    (setq marker-name "cable_front")
    (when (send *tfl* :wait-for-transform "dual_arm_base" marker-name stamp 5)
      (setq base-to-marker-origin (send *tfl* :lookup-transform "dual_arm_base" marker-name stamp)))
    (setq base-to-pre-grasp-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *cable-front-to-pre-grasp-pose*))
    (setq base-to-grasp-pose (send (send base-to-marker-origin :copy-worldcoords) :transform *cable-front-to-grasp-pose*))

    ;pre-grasp-pose
    (send *ri* :start-grasp :rarm)
    (unix:sleep 1)
    (send *ri* :stop-grasp :rarm :width 0.02)
    (unix:sleep 1)
    
    (send *robot* :rarm :inverse-kinematics base-to-pre-grasp-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    ;grasp-pose
    (send *robot* :rarm :inverse-kinematics base-to-grasp-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
    (send *ri* :wait-interpolation)

    (send *ri* :start-grasp :rarm)
    (unix:sleep 2)
    (send *ri* :stop-grasp :rarm)
    (unix:sleep 2)))


(defun check-camera ()
  (let ((base-to-marker-000 nil)
	(base-to-marker-001 nil)
	(base-to-marker-002 nil)
	(stamp nil)
	(ret nil))
    (ros::ros-info "grasp cable")
    (setq stamp (ros::time-now))
    (setq marker-name "cable_front")
    (when (send *tfl* :wait-for-transform "dual_arm_base" "test_000" stamp 5)
      (setq *marker-000* (send *tfl* :lookup-transform "dual_arm_base" "test_000" stamp)))
    (when (send *tfl* :wait-for-transform "dual_arm_base" "test_000" stamp 5)
      (setq *marker-001* (send *tfl* :lookup-transform "dual_arm_base" "test_001" stamp)))
    (when (send *tfl* :wait-for-transform "dual_arm_base" "test_001" stamp 5)
      (setq *marker-002* (send *tfl* :lookup-transform "dual_arm_base" "test_002" stamp)))))
