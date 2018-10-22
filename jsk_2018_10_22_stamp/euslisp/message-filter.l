#!/usr/bin/env roseus


(defclass mask-centroid-synchronizer
  :super exact-time-message-filter)

(defmethod mask-centroid-synchronizer
  (:callback (image caminfo)
    (print (list image caminfo))
    (print (send-all (list image caminfo) :header :stamp))))


(ros::roseus "hoge")
(ros::roseus-add-msgs "sensor_msgs")
;; test
(setq hoge (instance image-caminfo-synchronizer :init
                     (list (list "/multisense/left/image_rect_color" sensor_msgs::Image)
                           (list "/multisense/left/camera_info" sensor_msgs::CameraInfo))))
(ros::spin)
