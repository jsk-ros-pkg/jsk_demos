#!/usr/bin/env roseus

#| 顔に描かれた四角形の面積を表示させる |#
(ros::roseus-add-msgs "jsk_recognition_msgs")

(defclass face_area
  :slots (*rect*)
  )
(defmethod face_area
  (:init ()
	 (ros::roseus "face_recognized_client")
	 (ros::subscribe "/edgetpu_face_detector/output/rects" jsk_recognition_msgs::RectArray #'send self :cb)
	 (setq *rect* nil)
	 (ros::ros-info "init")
	 self)
  (:cb (msg)
       (setq *rect* (send msg :rects))
       )
  (:loop ()
	 (ros::rate 10)
	 (while (ros::ok)
	   (if *rect*
	       (progn
		 (setq w (send (elt *rect* 0) :width))
		 (setq h (send (elt *rect* 0) :height))
		 (ros::ros-info "area: ~A" (* w h))
		 )
	     )
	     (ros::spin-once)
	     (ros::sleep)
	    )
	 (exit)
	 )
  )
