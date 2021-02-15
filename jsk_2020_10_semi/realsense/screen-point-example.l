#!/usr/bin/env roseus

(ros::roseus-add-srvs "jsk_recognition_msgs")
(ros::roseus "screen_point_example")

;; what is the 3D coords w.r.t camera_link of #(320 240) of camera_image
(setq x 320.0
      y 240.0)

;; call PointcloudScreenPointNodelet::screen_to_point
(setq s2p-srv "/pointcloud_screenpoint_nodelet/screen_to_point")
(ros::wait-for-service s2p-srv)
(ros::advertise "/camera/color/image_raw/screenpoint" geometry_msgs::PointStamped)

;; get 3D coords from image 2D point
(ros::rate 10)
(do-until-key
 ;; THIS is very important to update PointCloud2 marker
 (setq msg (instance geometry_msgs::PointStamped :init))
 (send msg :header :stamp (ros::time-now))
 (send msg :point :x x)
 (send msg :point :y y)
 (ros::publish "/camera/color/image_raw/screenpoint" msg)
 ;; use service call to get 3D point. Do not call this function BEFORE pub "/camera/color/image_raw/screenpoint"
 (setq req (instance jsk_recognition_msgs::TransformScreenpointRequest :init
                     :x x :y y))
 (setq res (ros::service-call s2p-srv req))
 ;; debug print
 (ros::ros-info "3D coords of ~A ~A is ~A ~A ~A"
                x y
                (send res :point :x)
                (send res :point :y)
                (send res :point :z))
 (ros::spin-once)
 (ros::sleep)
 ;;
 )






