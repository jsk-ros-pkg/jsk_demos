#!/usr/bin/env roseus
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::roseus "door_detection")

(defun checker-detec-callback (msg)
  (print "a")
  )



(ros::subscribe "/multi_plane_estimate/output_coefficients" jsk_recognition_msgs::ModelCoefficientsArray #'checker-detec-callback)

(do-until-key
 ;;(x::window-main-one)
 (ros::sleep)
 (ros::spin-once))
(exit)
