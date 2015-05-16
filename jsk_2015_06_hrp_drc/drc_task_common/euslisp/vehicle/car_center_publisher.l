#!/usr/bin/env roseus

(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "dynamic_tf_publisher")


(defun marker-pose-cb (msg)
  (setq c (ros::tf-pose-stamped->coords msg))
  (setq trans (ros::coords->tf-transform c))
  (setq req (instance dynamic_tf_publisher::SetDynamicTFRequest :init
                      :freq 1
                      :cur_tf (instance geometry_msgs::TransformStamped :init
                                        :header (instance std_msgs::Header :init
                                                          :stamp (ros::time 0)
                                                          :frame_id "BODY")
                                        :child_frame_id "car_marker"
                                        :transform trans)))
  (setq res (ros::service-call "/set_dynamic_tf" req))
  ;; (print-ros-msg res)
  )

;; main
(ros::roseus "tf_car_center_publisher")
(setq *tfl* (instance ros::transform-listener :init))
(setq *tfb* (instance ros::transform-broadcaster :init))
(ros::subscribe "car_pose_marker" geometry_msgs::PoseStamped #'marker-pose-cb)
(ros::rate 10)
(ros::spin)
