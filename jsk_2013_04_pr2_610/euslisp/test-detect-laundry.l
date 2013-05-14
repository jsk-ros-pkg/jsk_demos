#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(load "package://jsk_demo_common/euslisp/pr2-action.l")
(load "package://pr2eus/speak.l")


(ros::roseus "test_detect_laundry")

(ros::rate 1)
(pr2-init)
(speak-en "sentakuki")

(while (ros::ok)
  (setq laundry-coords (check-detection :type "laundry_button" :speak-name "sentakuki"))
  ;;(send *laundry* :move-to laundry-coords)
  ;;(send *pr2* :inverse-kinematics (send *laundory* :button-handle))
  (warn "laundry-coords: ~A~%" laundry-coords)
  (ros::spin-once)
  (ros::sleep))