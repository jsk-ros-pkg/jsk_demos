#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::load-ros-manifest "sensor_msgs")

(ros::roseus "marker_test")

(dual_panda-init)

(objects (list *robot*))
