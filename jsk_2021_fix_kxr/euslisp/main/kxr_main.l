#!/usr/bin/env roseus

(load "/home/amabe/prog/rcb4eus/rosrcb4.l")
(make-kxr-robot "kxrl2l5a3h2g")
(send *ri* :ros-open)
(send *ri* :timer-on)
(send *ri* :hold-all)
(init-publish)

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::load-ros-manifest "std_msgs")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "jsk_2021_fix_kxr")

(defun rarm-free ()
  (ros::ros-info "rarm free")
  (send *ri* :rarm-free))

(defun larm-free ()
  (ros::ros-info "larm free")
  (send *ri* :larm-free))

(defun rarm-hold ()
  (ros::ros-info "rarm hold")
  (send *ri* :rarm-hold))

(defun larm-hold ()
  (ros::ros-info "larm hold")
  (send *ri* :larm-hold))

(defun rleg-free ()
  (ros::ros-info "rleg free")
  (send *ri* :rleg-free))

(defun lleg-free ()
  (ros::ros-info "lleg free")
  (send *ri* :lleg-free))

(defun rleg-hold ()
  (ros::ros-info "rleg hold")
  (send *ri* :rleg-hold))

(defun lleg-hold ()
  (ros::ros-info "lleg hold")
  (send *ri* :lleg-hold))

(defun show-rarm-shoulder-r ()
  (ros::ros-info "show-rarm-shoulder-r")
  (send *robot* :angle-vector #f(0.0 -10.0 40.0 -20.0 0.0 0.0 -10.0 40.0 -20.0 0.0 -90.0 -90.0 0.0 -27.0 -27.0 -90.0 90.0 0.0 -27.0 -27.0 0.0 10.0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 200))

(defun show-rarm-elbow-p ()
  (ros::ros-info "show-rarm-elbow-p")
  (send *robot* :angle-vector #f(0.0 -10.0 40.0 -20.0 0.0 0.0 -10.0 40.0 -20.0 0.0 0.0 -90.0 0.0 -27.0 -27.0 0.0 90.0 0.0 -27.0 -27.0 0.0 10.0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 200))

(defun stand-pose ()
  (ros::ros-info "stand-pose")
  (send *robot* :angle-vector #f(0.0 -20.0 40.0 -20.0 0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 -30.0 0.0 -27.0 -27.0 0.0 30.0 0.0 -27.0 -27.0 0.0 10.0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 300))

(defun send-servo-info ()
  (ros::ros-info "send-servo-info")
  (setq msg (instance sensor_msgs::jointstate :init))
  (send msg :name (send *robot* :joint-access-names))
  (send *robot* :angle-vector (send *robot* :ri :read-angle-vector))
  (send msg :effort (check-timeout-servo :ri t :name (send *robot* :name)))
  (send msg :position (send *robot* :angle-vector))
  (send *robot* :ri :real-orientation)
  (send *robot* :ri :robot :angle-vector (send *robot* :angle-vector))
  (send *robot* :ri :robot :draw-torque
	((send *robot* :ri :viewer) . x::viewer)
	:torque-vector (send *robot* :ri :read-torque-vector)
	:flush nil :size 10.0)
  (send *robot* :ri :viewer :draw-objects :flush t)
  (send *robot* :ri :viewer :flush)
  (send *irtviewer* :draw-objects :flush t)
  msg)

(defun repair_reaction (req)
  (let ((m (send req :response))
	(f 0))
    (ros::ros-info (format nil "request is ~A" (send req :action)))
    (setq action (send req :action))
    (if (string= action "rarm-free")
	(progn
	  (rarm-free)
	  (setq f 1)))
    (if (string= action "larm-free")
	(progn
	  (larm-free)
	  (setq f 1)))
    (if (string= action "rarm-hold")
	(progn
	  (rarm-hold)
	  (setq f 1)))
    (if (string= action "larm-hold")
	(progn
	  (larm-hold)
	  (setq f 1)))
    (if (string= action "rleg-free")
	(progn
	  (rleg-free)
	  (setq f 1)))
    (if (string= action "lleg-free")
	(progn
	  (lleg-free)
	  (setq f 1)))
    (if (string= action "rleg-hold")
	(progn
	  (rleg-hold)
	  (setq f 1)))
    (if (string= action "lleg-hold")
	(progn
	  (lleg-hold)
	  (setq f 1)))
    (if (string= action "show-rarm-elbow-p")
	(progn
	  (show-rarm-elbow-p)
	  (setq f 1)))
    (if (string= action "show-rarm-sholder-r")
	(progn
	  (show-rarm-sholder-r)
	  (setq f 1)))
    (if (string= action "send-servo-info")
	(progn
	  (send m :joint_state (send-servo-info))
	  (setq f 1)))
    
    
    (if (= f 1)
	(progn
	  (send m :result "success"))
      (progn
	(send m :result "false")))
    m))

(ros::roseus "kxr_server")
(ros::advertise-service "repair_reaction" jsk_2021_fix_kxr::RepairInfo #'repair_reaction)
(do-until-key
 (ros::spin-once))
