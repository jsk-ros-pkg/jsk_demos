#!/usr/bin/env roseus

;; load package
(ros::load-ros-package "std_srvs")

;; load files
;; (require :pepper-interface "package://peppereus/pepper-interface.l")

;; var
(defvar *process-rate* 5)

;; parameter
(defparameter *continue* t)

(ros::roseus "lecture-demo")

(ros::subscribe "joy" sensor_msgs::Joy
		#'(lambda (msg)
		    (let (button)
		      ;; reset value
		      (setq *continue* t)
		      ;; update value
		      (setq button (send msg :buttons))

		      ;; A button: deai_1
		      (if (eq (elt button 0) 1)
			  (progn
			    (ros::wait-for-service "episode_11")
                            (call-empty-service "episode_11")
			    ))

		      ;; B button: deai_2
		      (if (eq (elt button 1) 1)
			  (progn
			    (ros::wait-for-service "episode_12")
                            (call-empty-service "episode_12")
			    ))
		      
		      ;; Logicool button: stop this node
		      (if (eq (elt button 8) 1)
			  (setq *continue* nil))
		      )))

(defun main ()
  (ros::rate *process-rate*)
  (print "start")
  (while *continue*
    (ros::spin-once)
    ;; (send *ri* :spin-once)
    (ros::sleep)
    ); while
  (print "end")
  )





