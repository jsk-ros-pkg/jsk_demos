#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(require "package://jsk_2021_fix_kxr/euslisp/grasp_cable/parameters.l")
(require "package://jsk_2021_fix_kxr/euslisp/grasp_cable/utils.l")

(ros::roseus-add-msgs "std_msgs")

(ros::load-ros-manifest "apriltag_ros")

(ros::roseus "grasp_cable")

(dual_panda-init)

(objects (list *robot*))


(send *robot* :reset-pose)
(send *robot* :rarm :move-end-pos #f(-500 0 100))
(send *robot* :larm :move-end-pos #f(-500 0 100))
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

;;before rarm grasp kxr
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(341.012 -147.428 1370.354) :rpy (float-vector 1.222 1.551 -0.386))) :translation-axis t :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(227.932 139.233 1435.999) :rpy (float-vector -0.09 1.312 2.977))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)


;;lower rarm
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(341.012 -147.428 1301.354) :rpy (float-vector 1.222 1.551 -0.386))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;grasp kxr
(send *ri* start-grasp :rarm)
(unix:sleep 1)

;;grasp elbow from below
(larm-grasp-rarm-elbow-p-servo-from-below)
