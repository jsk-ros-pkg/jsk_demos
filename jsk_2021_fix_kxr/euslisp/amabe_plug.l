#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")

(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")

(dual_panda-init)

(objects (list *robot*))

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(431 -219.735 1100) :rpy (float-vector 0.0 1.57 0.0))) :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(413 340 1421) :rpy (float-vector -1.57 1.57 1.57))) :rotation-axis t)

(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(unix::sleep 2)

(send *robot* :rarm :move-end-pos #f(21 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)
(send *robot* :rarm :move-end-pos #f(-40 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

#|
(send *ri* :start-grasp :rarm)
(unix::sleep 2)
(send *ri* :stop-grasp :rarm)
(send *ri* :start-grasp :rarm)
(unix::sleep 1)
(send *ri* :start-grasp :rarm)
(unix::sleep 1)
(send *ri* :start-grasp :rarm)
(unix::sleep 1)
|#
