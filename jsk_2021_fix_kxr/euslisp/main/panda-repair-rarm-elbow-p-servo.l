#!/usr/bin/env roseus

(defun panda-repair-rarm-elbow-p-servo ()
  (ros::ros-info "Start repairing rarm-elbow-p-servo")

  (setq search-tag-pose (send (send *base_to_rarm_elbow_p_servo* :copy-worldcoords) :transform *servo-to-search-cable-pose*))
  (send *robot* :rarm_joint1 :joint-angle 30)
  (send *robot* :rarm :inverse-kinematics search-tag-pose)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (rarm-search-servo-tag "rpov-rarm_elbow_p_servo")

  (unix:sleep 2)
  (setq stamp (ros::time-now))
  (when (send *tfl* :wait-for-transform "dual_arm_base" "rpov-rarm_elbow_p_servo" stamp 5)
    (setq *base_to_rarm_elbow_p_servo* (send *tfl* :lookup-transform "dual_arm_base" "rpov-rarm_elbow_p_servo" stamp)))

  (send *ri* :start-grasp :larm)

  (while t
    (rarm-arange-cable "rpov-rarm_elbow_p_servo" "right")
    
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
	      (ros::ros-info "Failed to insert cable")
	      (ros::ros-info "Retry to insert cable"))
	  (progn
	    (ros::ros-info "Success to insert cable")
	    (return-from nil))
	  )
	)
      )
    )

  (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(300.472 -75.155 1205.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
  (send *robot* :rarm :move-end-pos #f(-100 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm)
  (unix:sleep 1)

  (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(300.472 -75.155 1205.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
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

  (ros::ros-info "End repairing rarm-elbow-p-servo"))
