#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")

(unless (boundp '*ri*) (setq *ri* (instance spot-interface :init)))
(ros::load-ros-manifest "jsk_recognition_msgs")

(defun check-human ()
  (ros::roseus "check-human")
  (ros::rate 10)
  (setq f 0)
  (while (= f 0)
    (setq rects (send (one-shot-subscribe "/edgetpu_human_pose_estimator/output/rects" jsk_recognition_msgs::RectArray) :rects))
    ;;spotの関節角度司令用
    (let ((spot_angle (instance std_msgs::UInt16 :init)))
      (when rects
    	(progn
    	  (setq size (* (send (elt rects 0) :x) (send (elt rects 0) :y)))
    	  (ros::ros-info "size is ~A" size)
    	  (if (> size 60000)
    	      (progn (ros::ros-info "found human")
		     (send *ri* :body-pose 0.3 0 0)
    		     (send *ri* :speak-jp "上のかごに載せてください")
    		     (setq f 1))))
    	)
      )
    )
  )
