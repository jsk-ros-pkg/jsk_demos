#!/usr/bin/env roseus

(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "dynamic_tf_publisher")

(defun marker-pose-cb (msg)
  (let ((trans-body-to-camera (send *tfl* :lookup-transform "BODY"
                                    (send msg :header :frame_id)
                                    (send msg :header :stamp)
                                    )
                              )
        body-to-car-marker
        )
    (unless trans-body-to-camera
      (warn "Failed to get car_marker~%")
      (return-from marker-pose-cb nil)
      )

    (setq c (ros::tf-pose-stamped->coords msg))
    (setq body-to-car-marker
          (send trans-body-to-camera :transform c)
          )
    (setq trans (ros::coords->tf-transform body-to-car-marker))
    (setq req (instance dynamic_tf_publisher::SetDynamicTFRequest :init
                        :freq 10
                        :cur_tf (instance geometry_msgs::TransformStamped :init
                                          :header (instance std_msgs::Header :init
                                                            :stamp (send msg :header :stamp)
                                                            :frame_id "BODY")
                                          :child_frame_id "car_marker"
                                          :transform trans)))
    (setq res (ros::service-call "/set_dynamic_tf" req))
    ;; (warn "car_marker is subscribed~%")
    )
  ;; (print-ros-msg res)
  )

;; main
(ros::roseus "tf_car_center_publisher")
(setq *tfl* (instance ros::transform-listener :init))
(ros::subscribe "car_pose_marker" geometry_msgs::PoseStamped #'marker-pose-cb)
(setq c (make-coords :pos (float-vector 439.95 -451.099 433.05) :rpy (float-vector 3.083 1.039 -3.094))) ;; hrp2
(setq trans (ros::coords->tf-transform c))
(setq req (instance dynamic_tf_publisher::SetDynamicTFRequest :init
                    :freq 10
                    :cur_tf (instance geometry_msgs::TransformStamped :init
                                      :header (instance std_msgs::Header :init
                                                        :stamp (ros::time 0)
                                                        :frame_id "BODY")
                                      :child_frame_id "car_marker"
                                      :transform trans)))
(setq res (ros::service-call "/set_dynamic_tf" req))
(ros::rate 10)
(ros::spin)
