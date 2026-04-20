#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")
;;(ros::roseus-add-msgs "geometry_msgs")
#|
;; これはpublish
;;(ros::advertise "/edgetpu_face_detector/output/rects" jsk_recognition_msgs:;;:RectArray 1)

;;(unix::sleep 1)
;;(ros::duration-sleep 1)
;;(print (ros::get-num-subscribers "/edgetpu_face_detector/output/rects"))
|#

;; 大域変数として定義

;;(setq *rect* (instance jsk_recognition_msgs::RectArray :init))
(setq *rect* nil)


(defun cb (msg)
  (if msg
      (setq *rect* (send msg :rects))
    )
)
;;main
(ros::roseus "face_rect_client")
(ros::subscribe "/edgetpu_face_detector/output/rects" jsk_recognition_msgs::RectArray #'cb)

(ros::rate 10)
(while (ros::ok)
  (if *rect*
      (progn
	;;(ros::ros-info "hello")
	;;(ros::ros-info "~A" (send (car *rect*))
	(ros::ros-info "x: ~A" (send (car *rect*) :x))
	(ros::ros-info "y: ~A" (send (car *rect*) :y))
	(ros::ros-info "width: ~A" (send (car *rect*) :width))
	(ros::ros-info "height: ~A" (send (car *rect*) :height))

	;; これらは見慣れない書き方らしい
	;;(setq w (send (car *rect*) :width)) 
	;;(setq h (send (car *rect*) :height))

	;; 良い書き方
	(setq w (send (elt *rect* 0) :width))
	(setq h (send (elt *rect* 0) :height))
	(ros::ros-info "area : ~A" (* w h))
	)
    )
  (ros::spin-once)
  (ros::sleep)
  )
