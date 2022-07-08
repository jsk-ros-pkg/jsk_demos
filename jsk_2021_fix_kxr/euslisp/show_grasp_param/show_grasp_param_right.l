#!/usr/bin/env roseus

(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")

(ros::roseus "show-grasp-param-right")

;; (ros::subscribe "/rarm_vision/camera/color/image_raw" image_view2::ImageMarker2 #'cb)


;; (defun cb (msg)
;;   (let (mrk (instance image_view2::ImageMarker2 :init))
;;     (send mrk :type image_view2::ImageMarker2::*TEXT*)
;;     (send mrk :position (instance geometry_msgs::Point :init :x 320 :y 40))
;;     (send mrk :scale 2.5) ;; this is proocol
;;     (send mrk :fill_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 0.5))
;;     (send mrk :text "Which shelf?")
;;     (ros::publish "/image_marker" mrk)
;;     ))

(ros::advertise "image_marker" image_view2::ImageMarker2 1)

(ros::rate 10)
(while (ros::ok)
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :type image_view2::ImageMarker2::*POLYGON*)
    (send mrk :points (list (instance geometry_msgs::Point :init :x 630 :y 630)
			    (instance geometry_msgs::Point :init :x 710 :y 630)
			    (instance geometry_msgs::Point :init :x 710 :y 690)
			    (instance geometry_msgs::Point :init :x 630 :y 690)))
    (ros::publish "image_marker" mrk)
    (ros::spin-once)
    (ros::sleep)
    ))
