#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")

(ros::roseus "arduino_client")

(defun arduino-cb (msg) (print (list 'cb (send msg :data))))
(ros::subscribe "arduino" std_msgs::Int16 #'arduino-cb)

(ros::spin)
(exit)
