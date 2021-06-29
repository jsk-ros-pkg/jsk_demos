#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")

(dual_panda-init)

(send *ri* :start-grasp :rarm)
(unix::sleep 1)

(exit)
