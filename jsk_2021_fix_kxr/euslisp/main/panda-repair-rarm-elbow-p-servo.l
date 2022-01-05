#!/usr/bin/env roseus

(defun panda-repair-rarm-elbow-p-servo ()
  (ros::ros-info "Start repairing rarm-elbow-p-servo")
  
  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "show-rarm-elbow-p")
  (setq res (ros::service-call "repair_reaction" req t))
  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "rarm-free")
  (setq res (ros::service-call "repair_reaction" req t))
  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "larm-free")
  (setq res (ros::service-call "repair_reaction" req t))

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
  (while t
    (rarm-grasp-cable "rpov-rarm_elbow_p_servo" "right")

    (rarm-insert-cable "rpov-rarm_elbow_p_servo" "right")

    (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
    (send req :action "send-servo-info")
    (setq res (ros::service-call "repair_reaction" req t))
    (setq joint_states (send res :joint_state))
    (setq timeout-servo-list (make-timeout-servo-list joint_states))
    (if (not timeout-servo-list)
	(progn
	  (ros::ros-info "Success to insert cable")
	  (return-from nil))
      (progn
	(setq target (decide-target-servo timeout-servo-list))
	(if (string-equal target "rarm-elbow-p")
	    (progn
	      (ros::ros-info "Success to insert cable")
	      (return-from nil))
	  (progn
	    (ros::ros-info "Failed to insert cable")
	    (ros::ros-info "Retry to insert cable"))))))

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
  )

  (ros::ros-info "End repairing rarm-elbow-p-servo"))
