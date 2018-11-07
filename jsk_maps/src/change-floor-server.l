#!/usr/bin/env roseus


(load "package://jsk_maps/src/change-floor.l")

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "std_msgs")
(ros::roseus "change-floor-server")


(defun change-floor-callback (msg)
  (let ((target-floor-name (send msg :data)))
    (ros::ros-info
      (format nil "floor changed to ~A" target-floor-name))
    (change-floor target-floor-name)))


(setq base-frame-name (ros::get-param "~base_frame_id" "base_footprint"))
(setq topicname (ros::get-param "~initialpose" "/initialpose3d"))
(setq map-tf-mux (ros::get-param "~map_tf_mux" "/map_tf_mux/selected"))

(ros::subscribe "~input" std_msgs::String
  #'change-floor-callback)

(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (ros::sleep))

(exit)
