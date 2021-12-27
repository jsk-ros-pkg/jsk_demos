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
(send *robot* :angle-vector (send *ri* :state :potentio-vector))

(send *robot* :reset-coop-pose)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(while t
  (setq found-body (search-body))
  (if (string= found-body "front")
      (progn
	(ros::ros-info "Try to reverse body")))
  (if (string= found-body "back")
      (progn
	(ros::ros-info "Recordind position of kxr")
	(setq stamp (ros::time-now))
	(when (send *tfl* :wait-for-transform "dual_arm_base" "rpov-body_back" stamp 5)
	  (setq *base_to_body_back* (send *tfl* :lookup-transform "dual_arm_base" "rpov-body_back" stamp)))
	(return))))

(setq base-to-larm-grasp-pose (send (send *base_to_body_back* :copy-worldcoords) :transform *body-back-to-larm-grasp-pose*))
(setq base-to-rarm-grasp-pose (send (send *base_to_body_back* :copy-worldcoords) :transform *body-back-to-rarm-grasp-pose*))

(send *robot* :rarm :inverse-kinematics base-to-larm-grasp-pose)
(send *robot* :rarm :move-end-pos #f(-100 0 0))
(send *robot* :larm :inverse-kinematics base-to-rarm-grasp-pose)
(send *robot* :larm :move-end-pos #f(-100 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)

(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
(unix:sleep 2)

(send *robot* :rarm :inverse-kinematics base-to-larm-grasp-pose)
(send *robot* :larm :inverse-kinematics base-to-rarm-grasp-pose)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(send *ri* :start-grasp :larm)
(unix:sleep 2)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(308.472 -259.081 1220.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(305.157 -14.307 1220.274) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(300.472 -75.155 1220.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(297.605 169.619 1220.223) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(300.472 -75.155 1210.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(297.605 169.619 1214.223) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :larm)
(unix:sleep 1)

(send *robot* :larm :move-end-pos #f(-100 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 2500)
(send *ri* :wait-interpolation)

(larm-search-rarm-elbow-p-servo-tag)

;;grasp elbow from below
(larm-grasp-rarm-elbow-p-servo-from-below)

(send *ri* :stop-grasp :rarm)
(unix:sleep 1)

(send *robot* :rarm :move-end-pos #f(-100 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

;;prepare to grasp cable
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(270.152 15.464 1414.154) :rpy (float-vector 2.5 1.551 -0.386))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(setq search-tag-pose (send (send *base_to_rarm_elbow_p_servo* :copy-worldcoords) :transform *servo-to-search-cable-pose*))
(send *robot* :rarm_joint1 :joint-angle 30)
(send *robot* :rarm :inverse-kinematics search-tag-pose)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(rarm-search-servo-tag "rpov-rarm_elbow_p_servo")

(setq stamp (ros::time-now))
(when (send *tfl* :wait-for-transform "dual_arm_base" "rpov-rarm_elbow_p_servo" stamp 5)
  (setq *base_to_rarm_elbow_p_servo* (send *tfl* :lookup-transform "dual_arm_base" "rpov-rarm_elbow_p_servo" stamp)))

;;<TODO recognize which cable is missing>

(rarm-grasp-cable "rpov-rarm_elbow_p_servo" "right")

(rarm-insert-cable "rpov-rarm_elbow_p_servo" "right")

(send *ri* :stop-grasp :rarm :width 0.007)
(unix:sleep 1)
(send *robot* :rarm :move-end-pos #f(-30 0 30))
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :move-end-pos #f(-70 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(300.472 -75.155 1210.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
(send *robot* :rarm :move-end-pos #f(-100 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(300.472 -75.155 1210.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(unix:sleep 1)

(larm-release-rarm-elbow-p-servo-from-below)

(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(297.605 169.619 1214.223) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
(send *robot* :larm :move-end-pos #f(-100 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(297.605 169.619 1214.223) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :larm)
(unix:sleep 1)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(308.472 -259.081 1220.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(305.157 -14.307 1220.274) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;;KXR stand up
