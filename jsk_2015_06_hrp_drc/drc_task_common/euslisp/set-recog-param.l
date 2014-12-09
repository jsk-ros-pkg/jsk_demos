#!/usr/bin/env roseus

(ros::roseus "set_recog_param")
(ros::load-ros-manifest "std_srvs")

(defun set-recog-param-init
  ()
  (ros::advertise-service "set_recog_param/valve_param" std_srvs::Empty #'set-recog-param-for-valve)
  (ros::advertise-service "set_recog_param/drill_param" std_srvs::Empty #'set-recog-param-for-drill)
  (ros::advertise-service "set_recog_param/handle_param" std_srvs::Empty #'set-recog-param-for-handle)
  )

(defun set-recog-param-main
  ()
  (do-until-key
   (ros::spin-once))
  )


(defun set-recog-param-for-valve
  (req)
  (set-recog-param-common "valve_param")
  (send req :response))

(defun set-recog-param-for-drill
  (req)
  (set-recog-param-common "drill_param")
  (send req :response))

(defun set-recog-param-for-handle
  (req)
  (set-recog-param-common "handle_param")
  (send req :response))


(defun set-recog-param-common
  (namespace)

  (ros::set-dynamic-reconfigure-param "/multi_plane_estimate" "connect_distance_threshold" :double (ros::get-param (format nil "~~~a/multi_plane_estimate/connect_distance_threshold" namespace) 0.2))
  (ros::set-dynamic-reconfigure-param "/multi_plane_estimate" "distance_threshold" :double (ros::get-param (format nil "~~~a/multi_plane_estimate/distance_threshold" namespace) 0.2))
  (ros::set-dynamic-reconfigure-param "/multi_plane_estimate" "min_size" :int (ros::get-param (format nil "~~~a/multi_plane_estimate/min_size" namespace) 200))
  (ros::set-dynamic-reconfigure-param "/plane_extraction" "min_height" :double (ros::get-param (format nil "~~~a/plane_extraction/min_height" namespace) 0.2))
  (ros::set-dynamic-reconfigure-param "/plane_extraction" "max_height" :double (ros::get-param (format nil "~~~a/plane_extraction/max_height" namespace) 0.2))
  (ros::set-dynamic-reconfigure-param "/euclidean_clustering" "tolerance" :double (ros::get-param (format nil "~~~a/euclidean_clustering/tolerance" namespace) 0.2))
  )


(set-recog-param-init)
;;(set-recog-param-main)
