#!/usr/bin/env roseus

(defun panda-release-kxr ()
  (ros::ros-info "Releasing KXR")

  (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(308.472 -259.081 1220.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
  (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(305.157 -14.307 1220.274) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *robot* :rarm :move-end-pos #f(-180 0 0))
  (send *robot* :larm :move-end-pos #f(-180 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *robot* :rarm :move-end-pos #f(-50 -50 0))
  (send *robot* :larm :move-end-pos #f(-50 50 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *robot* :rarm :move-end-pos #f(40 0 0))
  (send *robot* :larm :move-end-pos #f(40 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm)
  (send *ri* :stop-grasp :larm)
  (unix:sleep 2)

  (send *robot* :rarm :move-end-pos #f(-50 0 0))
  (send *robot* :larm :move-end-pos #f(-50 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *robot* :rarm :move-end-pos #f(0 100 0))
  (send *robot* :larm :move-end-pos #f(0 -100 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "rarm-hold")
  (setq res (ros::service-call "repair_reaction" req t))
  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "larm-hold")
  (setq res (ros::service-call "repair_reaction" req t))
  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "stand-pose")
  (setq res (ros::service-call "repair_reaction" req t))
  )
