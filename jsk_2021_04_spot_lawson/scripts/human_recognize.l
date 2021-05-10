#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")

(unless (boundp '*ri*) (setq *ri* (instance spot-interface :init)))
(ros::load-ros-manifest "jsk_recognition_msgs")

(defun check-human ()
  (ros::roseus "check-human")
  (ros::rate 10)
  (setq f 0)
  (setq th 250000)
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
	   (if (> size_b size) (progn
				 (setq deg (/ (* (- 1770 (send (elt rects i) :x)) 180) 1770))
				 (setq size size_b)))
	   (setq i (+ i 1)))
    	  (ros::ros-info "size is ~A, deg is ~A" size deg)
    	  (if (and (> size th) (> deg -90) (< deg 90))
    	      (progn (ros::ros-info "found human")
		     (send *ri* :go-pos 0 0 deg)
		     (unix::sleep 3)
		     (send *ri* :body-pose 0 0.3 0)
		     (send *ri* :speak-jp "ICカードで支払います")
		     (unix::sleep 8)
    		     (send *ri* :speak-jp "上のかごに載せてください")
		     (unix::sleep 8)
		     (send *ri* :body-pose 0 0 0)
		     (send *ri* :speak-jp "ありがとうございました")
		     (unix::sleep 2)
    		     (setq f 1)))))))
	  )


