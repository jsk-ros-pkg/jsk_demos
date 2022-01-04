#!/usr/bin/env roseus

(load "/home/amabe/prog/rcb4eus/rosrcb4.l")
(make-kxr-robot "kxrl2l5a3h2g")
(send *ri* :ros-open)
(send *ri* :timer-on)
(send *ri* :hold-all)
(init-publish)

(ros::roseus "kxr_robot")

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

(defun repair_reaction (req)
  (let ((m (send req :response))
	(f 0))
    (ros::ros-info (format nil "request is ~A" (send req :action)))
    (setq action (send req :action))
    (if (string= action "rarm-free")
	(progn
	  (rarm-free)
	  (send m :result "success")
	  (setq f 1)))
    (if (string= action "larm-free")
	(progn
	  (larm-free)
	  (send m :result "success")
	  (setq f 1)))
    (if (string= action "rarm-hold")
	(progn
	  (rarm-hold)
	  (send m :result "success")
	  (setq f 1)))
    m))

(ros::roseus "kxr_server")
(ros::advertise-service "repair_reaction" jsk_2021_fix_kxr::RepairInfo #'repair_reaction)
(do-until-key
 (ros::spin-once))


;; (defun servo-onoff()
;;   (setq msg (send (one-shot-subscribe "/panda_order" std_msgs::String) :data))
;;   (print msg)
;;   (cond ((string-equal msg "rarm-free")
;; 	 (progn
;; 	   (send *ri* :rarm-free)))
;; 	((string-equal msg "larm-free")
;; 	 (progn
;; 	   (send *ri* :larm-free)))
;; 	((string-equal msg "rarm-hold")
;; 	 (progn
;; 	   (send *ri* :rarm-hold)))
;; 	((string-equal msg "larm-hold")
;; 	 (progn
;; 	   (send *ri* :larm-hold)))))
