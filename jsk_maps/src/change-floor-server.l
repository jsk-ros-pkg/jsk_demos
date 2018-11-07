#!/usr/bin/env roseus


(load "package://jsk_maps/src/change-floor.l")

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "std_msgs")
(ros::roseus "change-floor-server")

(defun change-floor-callback (msg)
  "get std_msgs::String and call change-floor function to
  change map tf frame."
  (let ((target-floor-name (send msg :data)))
    (ros::ros-info
      (format nil "floor changed to ~A" target-floor-name))
    (change-floor target-floor-name)))

;; initlalize tf listener
(setq *tfl* (instance ros::transform-listener :init))

;; get parameter
(setq base-frame-name (ros::get-param "~base_frame_id" "base_footprint"))
(setq topicname (ros::get-param "~initialpose" "/initialpose3d"))
(setq map-tf-mux (ros::get-param "~map_tf_mux" "/map_tf_mux/selected"))

;; initlalize subscriber
(ros::subscribe "~input" std_msgs::String
  #'change-floor-callback)

;; main loop
(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (ros::sleep))

(exit)
