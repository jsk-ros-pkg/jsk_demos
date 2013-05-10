#!/usr/bin/env roseus

(load "package://jsk_demo_common/euslisp/pr2-action.l")

(ros::init)

(ros::rate 1)

(while (ros::ok)
  (setq laundry-coords (check-detection :type "laundry_center" :speak-name "せんたくき"))
  (if laundry-coords (ros::ros-info "laundry-coords: ~A~%" laundry-coords))
  (ros::spin-once)
  (ros::sleep))