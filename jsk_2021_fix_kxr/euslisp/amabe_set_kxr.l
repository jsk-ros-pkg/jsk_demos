#!/usr/bin/env roseus

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

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(513 -88 1200) :rpy (float-vector -1.57 1.57 0.0))) :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(513 177 1200) :rpy (float-vector -1.57 1.57 0.0))) :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(513 -88 1093.682) :rpy (float-vector -1.57 1.57 0.0))) :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(513 177 1093.682) :rpy (float-vector -1.57 1.57 0.0))) :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(send *ri* :start-grasp :larm)
(unix::sleep 2)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(513 -138 1093.682) :rpy (float-vector -1.57 1.57 0.0))) :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(513 127 1093.682) :rpy (float-vector -1.57 1.57 0.0))) :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
(unix::sleep 2)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(513 -138 1300.682) :rpy (float-vector -1.57 1.57 0.0))) :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(513 127 1300.682) :rpy (float-vector -1.57 1.57 0.0))) :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(513 -310 1300.682) :rpy (float-vector -1.57 1.57 0.0))) :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(510 -310 1170.364) :rpy (float-vector 1.57 0.0 -3.14))) :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(510 -310 1070.364) :rpy (float-vector 1.57 0.0 -3.14))) :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(510 -260 1070.364) :rpy (float-vector 1.57 0.0 -3.14))) :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)
(unix::sleep 2)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(510 -260 1070.364) :rpy (float-vector 1.57 0.0 -1.57))) :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)
(unix::sleep 2)

(send *robot* :rarm :move-end-pos #f(-50 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)
