#!/usr/bin/env roseus
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::roseus "door_detection")
(setq *normal-vec* nil)

(defun door-detec-callback (msg)
  (setq *normal-vec* (send (car (send msg :coefficients)) :values))
  (setq normal-vec-y (elt *normal-vec* 1))
  (print normal-vec-y)
  (when (and (<  normal-vec-y 0.05) (> normal-vec-y -0.05)) 
    (print (send (car (send msg :coefficients)) :values))
    ;;(send *ri* )
    )
  )

;;(defun cb (msg)
;;  (setq *poly* (send (car (send msg :polygons)) :polygon))
;;  (setq poly (car (send *poly* :points)))
  ;;(print poly)
;;  )
;;(defun front-door-pose nil
;;  (send *


(ros::subscribe "/multi_plane_estimate/output_coefficients" jsk_recognition_msgs::ModelCoefficientsArray #'door-detec-callback)
;;(ros::subscribe "/multi_plane_estimate/output_polygon" jsk_recognition_msgs::PolygonArray #'cb)
(do-until-key
 ;;(x::window-main-one)
 (ros::sleep)
 (ros::spin-once))
(exit)
