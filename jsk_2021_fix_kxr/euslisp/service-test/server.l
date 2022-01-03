#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_2021_fix_kxr")

(defun hoge ()
  (ros::ros-info "hoge"))

(setq time 0)
(ros::ros-info (format nil "time : ~d" time))
(defun repair_reaction (req)
  (let ((m (send req :response))
	(f "success"))
    (ros::ros-info (format nil "request is ~A" (send req :action)))
    (setq action (send req :action))
    (if (string= action "hoge")
	(send m :result "hoge")
      (send m :result "huga"))
    m))

(ros::roseus "server")
(ros::advertise-service "repair_reaction" jsk_2021_fix_kxr::RepairInfo #'repair_reaction)
(do-until-key
 (ros::spin-once))


;; (ros::load-ros-manifest "roseus")
;; (defun add-two-ints (req)
;;   (let ((m (send req :response)))
;;     (format *error-output* "Returning [~d + ~d = ~d]"
;; 	    (send req :a) (send req :b)
;; 	    (+ (send req :a) (send req :b)))
;;     (send m :sum (+ (send req :a) (send req :b)))
;;     m))

;; (ros::roseus "add_two_ints_server")
;; (ros::advertise-service "add_two_ints" roseus::AddTwoInts #'add-two-ints)
;; (do-until-key
;;  (ros::spin-once))

