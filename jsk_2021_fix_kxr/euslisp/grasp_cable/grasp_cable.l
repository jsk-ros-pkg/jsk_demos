#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(require "package://jsk_2021_fix_kxr/euslisp/grasp_cable/parameters.l")
(require "package://jsk_2021_fix_kxr/euslisp/grasp_cable/utils.l")

(ros::roseus-add-msgs "std_msgs")

(ros::load-ros-manifest "apriltag_ros")

(ros::roseus "grasp_cable")

(dual_panda-init)

(objects (list *robot*))

(defvar *marker-000*)
(defvar *marker-001*)
(defvar *marker-002*)

(send *robot* :reset-pose)
(send *robot* :rarm :move-end-pos #f(-500 0 100))
(send *robot* :larm :move-end-pos #f(-500 0 100))
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

;;before rarm grasp kxr
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(277.724 510.724 1373.929) :rpy (float-vector -0.827 1.457 2.009))) :translation-axis t :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(341.18 -136.643 1312.764) :rpy (float-vector -0.031 1.556 -1.602))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;;lower rarm
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(277.724 510.724 1443.929) :rpy (float-vector -0.827 1.457 2.009))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* start-grasp :rarm)
(unix:sleep 1)
