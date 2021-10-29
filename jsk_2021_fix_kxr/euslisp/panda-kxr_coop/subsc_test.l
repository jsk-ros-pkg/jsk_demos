#!/usr/bin/env roseus

(ros::roseus "kxr_robot")

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::load-ros-manifest "std_msgs")
(ros::load-ros-manifest "sensor_msgs")

(defvar *panda_order*)

(defun order-cb (msg)
  (let ((order (instance std_msgs::String :init) hoge))
    (setq hoge (send msg :data))
    (setq *panda_order* (send msg :data))
    (ros::ros-info "~A" hoge)))
    
(ros::subscribe "/panda_order" std_msgs::String #'order-cb)

(ros::rate 1)
(do-until-key
 (ros::subscribe "/panda_order" std_msgs::String #'order-cb)
 (print "hoge")
 (print *panda_order*)
 (ros::spin-once)
 (ros::sleep))

(while (ros::ok)
 (ros::subscribe "/panda_order" std_msgs::String #'order-cb)
 (print "hoge")
 (print *panda_order*)
 (ros::spin-once)
 (ros::sleep))
