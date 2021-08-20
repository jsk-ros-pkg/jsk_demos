#!/usr/bin/env roseus


(load "package://jsk_maps/src/change-floor.l")

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "std_msgs")
(ros::roseus "change-floor-server")


;; initlalize tf listener
(setq *tfl* (instance ros::transform-listener :init))

;; get parameter
(setq *move-base-node* (ros::get-param "~move_base_node" "move_base_node"))
(setq *costmap-name* (ros::get-param "~costmap_name" "global_costmap"))
(setq *base-frame-name* (ros::get-param "~base_frame_id" "base_footprint"))
(setq *topicname* (ros::get-param "~initialpose" "/initialpose3d"))


(defun change-floor-callback (msg)
  "get std_msgs::String and call change-floor function to
  change map tf frame."
  (let ((target-floor-name (send msg :data)))
    (ros::ros-info
      (format nil "floor changed to ~A" target-floor-name))
    (change-floor target-floor-name
                  :topicname *topicname*
                  :base-frame-name *base-frame-name*)
    ;; change publish_freuquency to reload costmap automatically
    (let ((current-costmap-publish-frequency
           (ros::get-param (format nil "~A/~A/publish_frequency"
                                   *move-base-node* *costmap-name*))))
      (unix:sleep 1)
      (change-costmap-publish-frequency 1.0
                                        :node-name *move-base-node*
                                        :costmap-name *costmap-name*)
      (unix:sleep 2)
      (change-costmap-publish-frequency current-costmap-publish-frequency
                                        :node-name *move-base-node*
                                        :costmap-name *costmap-name*))))


;; initlalize subscriber
(ros::subscribe "~input" std_msgs::String
  #'change-floor-callback)

;; main loop
(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (ros::sleep))

(exit)
