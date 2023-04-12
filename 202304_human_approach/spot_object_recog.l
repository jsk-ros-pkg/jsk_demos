#!/usr/bin/env roseus

(ros::load-ros-package "jsk_recognition_msgs")
(ros::roseus "test")

(defun cb (msg)
  (if (not (= (length (send msg :rects)) 0))
      (print (send (car (send msg :rects)) :x))))
      ;;(print (send (car (send msg :rects)) :y))))

(ros::subscribe "/human" jsk_recognition_msgs::RectArray #'cb)
(ros::spin)
