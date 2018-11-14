#!/usr/bin/env roseus
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::roseus "door_detection")

(defun door-detec-callback (msg)
  (print (send (car (send msg :coefficients)) :values)))

(ros::subscribe "/multi_plane_estimate/output_coefficients" jsk_recognition_msgs::ModelCoefficientsArray #'door-detec-callback)

