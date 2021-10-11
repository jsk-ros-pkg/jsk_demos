#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")

(dual_panda-init)

(objects (list *robot*))

(send *robot* :angle-vector (send *ri* :state :potentio-vector))

(send *robot* :rarm :move-end-pos #f(12 0 0))

(send *ri* :angle-vector (send *robot* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm :width 0.004)
(unix::sleep 1)

(send *robot* :rarm :move-end-pos #f(-12 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(exit)
