#!/usr/bin/env roseus
(ros::load-ros-manifest "roseus")
(defun add-two-ints (req)
  (let ((m (send req :response)))
    (format *error-output* "Returning [~d + ~d = ~d]"
	    (send req :a) (send req :b)
	    (+ (send req :a) (send req :b)))
    (send m :sum (+ (send req :a) (send req :b)))
    m))

(ros::roseus "add_two_ints_server")
(ros::advertise-service "add_two_ints" roseus::AddTwoInts #'add-two-ints)
(do-until-key
 (ros::spin-once))


;; (ros::roseus "server_test")
;; (ros::roseus-add-srvs "jsk_2021_fix_kxr")
;; (ros::roseus-add-srvs "roseus")

;; (defun AddTwoInts
;;   (ros::advertise-service "addtwoints" roseus::AddTwoInts #'add-two-int
;;   )
