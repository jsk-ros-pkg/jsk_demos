#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")

(unless (boundp '*ri*) (setq *ri* (instance spot-interface :init)))
(ros::load-ros-manifest "jsk_recognition_msgs")

(defun check-human ()
  (ros::roseus "check-human")
  (ros::rate 10)
  (setq f 0)
  (setq th 180000)
  (while (= f 0)
    (setq rects (send (one-shot-subscribe "/edgetpu_panorama_human_pose_estimator/output/rects" jsk_recognition_msgs::RectArray) :rects))
    (let ((spot_angle (instance std_msgs::UInt16 :init)))
      (when rects
	(progn
	  (setq l (length rects))
	  (setq size 0)
	  (setq i 0)
	  (loop
	   (if (= i l) (return-from nil))
	   (setq size_b (* (send (elt rects i) :width) (send (elt rects i) :height)))
	   (if (> size_b size) (setq size size_b))
	   (setq i (+ i 1)))
    	  (ros::ros-info "size is ~A" size)
    	  (if (> size th)
    	      (progn (ros::ros-info "found human")
		     (send *ri* :body-pose 0 0.3 0)
		     (send *ri* :speak-jp "ICカードで支払います")
		     (unix::sleep 5)
    		     (send *ri* :speak-jp "上のかごに載せてください")
		     (unix::sleep 5)
		     (send *ri* :bod-pose 0 0 0)
		     (send *ri* :speak-jp "ありがとうございました")
		     (unix::sleep 2)
    		     (setq f 1)))))))
	  )


