#!/usr/bin/env roseus

(ros::roseus "button_state_marker")
(ros::roseus-add-msgs "visualization_msgs")

(defun text-marker (str &key (pose (make-coords)) (frame_id "base_link")
			(size 0.2) (color (float-vector 1 0 0)) (alpha 1.0))
  (let ((msg (instance visualization_msgs::marker :init)))
    (send msg :header :frame_id frame_id)
    (send msg :header :stamp (ros::time-now))
    (send msg :type visualization_msgs::tet_view_facing)
    (send msg :pose (ros::coords->tf-pose pose))
    (send msg :scale :z size)
    (send msg :color :r (elt color 0))
    (send msg :color :g (elt color 1))
    (send msg :color :b (elt color 2))
    (send msg :color :a alpha)
    (send msg :frame_locked t)
    (send msg :text str)
    msg))



(ros::roseus-add-msgs "image_view2")
(defun text-image-marker (str)
  (let ((msg (instance image_view2::ImageMarker2 :init)))
    (send msg :type image_view2::TEXT)
    (send msg :text str)
    msg ))


(setq *button-chk* '(nil nil))
(defun button-point-cb (msg) ;; Point
  (let (mrk (instance image_view2::ImageMarker2 :init))
    (send mrk :type image_view2::CIRCLE)
    (send mrk :position (send msg :pose :position))
    (send mrk :scale (send pos :z)) ;; this is proocol
    (send mrk :lifetime (ros::time 1.0))
    (ros::publish "/image_marker" mrk)
    ))
(defun button-light-cb (msg) ;; Float32
  (let ((mrk (text-image-marker (string (send msg :data)))))
    (send mrk :position :x )
    (send mrk :lifetime (ros::time 1.0))
    (ros::publish "/image_marker" mrk)
  ))

(defun elevator-number-cb (msg) ;; string
  (let ((mrk (text-image-marker (string (send msg :data)))))
    (send mrk :position :x 400)
    (send mrk :position :y 200)
    (send mrk :lifetime (ros::time 1.0))
    (ros::publish "/image_marker" mrk)
    ))

(defun object-detection-cb (msg) ;; ObjectDetection
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :lifetime (ros::time 1.0))
    (ros::publish "/image_marker" mrk)
  ))

(ros::advertise "/image_marker" image_view2::ImageMarker2)
