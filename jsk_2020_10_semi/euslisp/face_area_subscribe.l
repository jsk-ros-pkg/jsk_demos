#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(defclass face_area
  :slots (*rect*))
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
	 ;;(setq alpha 0.001)
	 (ros::rate 10)
	 (while (ros::ok)
	   ;;(setq alpha 0.001)
	   (if *rect*
	       (progn
		 (ros::ros-info "hello")
		 (ros::ros-info "width: ~A" (send (car *rect*) :width))
		 )
	     )
	   (ros::spin-once)
	   (ros::sleep)
	   )
	 (exit)
	 )
  )

(setq *ri* (instance face_area :init))
(send *ri* :loop)
