#!/usr/bin/env roseus

(defun panda-set-rleg ()
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

  (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(308.472 -259.081 1205.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
  (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(305.157 -14.307 1205.274) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :larm)
  (unix:sleep 1)

  (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(300.472 -75.155 1220.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
  (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(297.605 169.619 1220.223) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(300.472 -75.155 1210.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
  (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(297.605 169.619 1214.223) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm)
  (unix:sleep 1)

  (send *robot* :rarm :move-end-pos #f(-100 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2500)
  (send *ri* :wait-interpolation)

  ;;move-rarm
  )
