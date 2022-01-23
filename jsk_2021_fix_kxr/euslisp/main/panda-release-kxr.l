#!/usr/bin/env roseus

(defun panda-release-kxr ()
  (ros::ros-info "Releasing KXR")

  (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(358.472 -259.081 1230.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
  (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(355.157 -14.307 1230.274) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(358.472 -259.081 1220.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
  (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(355.157 -14.307 1220.274) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *robot* :rarm :move-end-pos #f(-130 0 0))
  (send *robot* :larm :move-end-pos #f(-130 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "before-stand-pose")
  (setq res (ros::service-call "repair_reaction" req t))
  
  (send *robot* :rarm :move-end-pos #f(-50 -50 0))
  (send *robot* :larm :move-end-pos #f(-50 50 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *robot* :rarm :move-end-pos #f(20 0 0))
  (send *robot* :larm :move-end-pos #f(20 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm)
  (send *ri* :stop-grasp :larm)
  (unix:sleep 2)

  (send *robot* :rarm :move-end-pos #f(-50 15 0))
  (send *robot* :larm :move-end-pos #f(-50 -15 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *robot* :rarm :move-end-pos #f(-50 130 0))
  (send *robot* :larm :move-end-pos #f(-50 -130 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "rarm-hold")
  (setq res (ros::service-call "repair_reaction" req t))
  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "larm-hold")
  (setq res (ros::service-call "repair_reaction" req t))
  (unix:sleep 2)
  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "stand-pose")
  (setq res (ros::service-call "repair_reaction" req t))
  (unix:sleep 2)
  (setq req (instance jsk_2021_fix_kxr::RepairInfoRequest :init))
  (send req :action "check-rarm-elbow-p")
  (setq res (ros::service-call "repair_reaction" req t))
  )

;; (send *ri* :stop-grasp :rarm)
;;   (send *ri* :stop-grasp :larm)
;;   (unix:sleep 1)
  
;;   (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(358.472 -259.081 1217.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
;;   (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(358.157 -14.307 1217.274) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
;;   (send *robot* :rarm :move-end-pos #f(-100 0 0))
;;   (send *robot* :larm :move-end-pos #f(-100 0 0))
;;   (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
;;   (send *ri* :wait-interpolation)

;;   (send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(358.472 -259.081 1205.164) :rpy (float-vector 1.57 1.526 0.0))) :translation-axis t :rotation-axis t)
;;   (send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(355.157 -14.307 1205.274) :rpy (float-vector -1.224 1.5 0.362))) :translation-axis t :rotation-axis t)
;;   (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
;;   (send *ri* :wait-interpolation)

;;   (send *ri* :start-grasp :rarm)
;;   (send *ri* :start-grasp :larm)
;;   (unix:sleep 1)
