#
!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")

(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")

(dual_panda-init)

(objects (list *robot*))
(send *robot* :reset-pose)
(send *robot* :rarm :move-end-pos #f(-500 0 0))
(send *robot* :larm :move-end-pos #f(-500 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(434 -14.541 1181.738) :rpy (float-vector 0.0 1.57 1.57))) :rotation-axis t)

(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(434 -14.541 1096) :rpy (float-vector 0.0 1.57 1.57))) :rotation-axis t)

(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(unix::sleep 2)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(434 -14.541 1181.738) :rpy (float-vector 0.0 1.57 1.57))) :rotation-axis t)

(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(434 -164.541 1181.738) :rpy (float-vector 0.0 1.57 1.57))) :rotation-axis t)

(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)


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
