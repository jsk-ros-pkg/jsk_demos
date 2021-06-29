#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")

(dual_panda-init)

(objects (list *robot*))

(send *ri* :stop-grasp :rarm :width 0.006)

(send *robot* :reset-pose)
(send *robot* :larm :move-end-pos #f(-500 0 0))
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(547 -18 823) :rpy (float-vector -3.14 1.57 1.57))) :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(do-until-key
 (print "waiting for Enter...")
 (unix::sleep 1))

(send *ri* :start-grasp :rarm :effort 100)
(unix::sleep 1)

(exit)
