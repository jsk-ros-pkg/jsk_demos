#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(require "package://jsk_2021_fix_kxr/euslisp/grasp_and_insert_cable/parameters.l")
(require "package://jsk_2021_fix_kxr/euslisp/grasp_and_insert_cable/utils.l")

(ros::roseus-add-msgs "std_msgs")

(ros::load-ros-manifest "apriltag_ros")

(ros::roseus "grasp_and_insert_cable")

(dual_panda-init)

(objects (list *robot*))

(send *robot* :reset-pose)
(send *robot* :rarm :move-end-pos #f(-500 0 100))
(send *robot* :larm :move-end-pos #f(-500 0 100))
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

;;before rarm grasp kxr
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(341.012 -147.428 1370.354) :rpy (float-vector 1.222 1.551 -0.386))) :translation-axis t :rotation-axis t)
(send *robot* :larm_joint1 :joint-angle -30)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(227.932 139.233 1435.999) :rpy (float-vector -0.09 1.312 2.977))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)


;;lower rarm
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(341.012 -147.428 1301.354) :rpy (float-vector 1.222 1.551 -0.386))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;;grasp kxr
(send *ri* :start-grasp :rarm)
(unix:sleep 1)

;;grasp elbow from below
(larm-grasp-rarm-elbow-p-servo-from-below)

(send *ri* :stop-grasp :rarm)

;;prepare to grasp cable
(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(341.012 -147.428 1430.354) :rpy (float-vector 1.222 1.551 -0.386))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(setq search-tag-pose (send (send *base-to-rarm-elbow-p-servo* :copy-worldcoords) :transform *servo-to-search-cable-pose*))
(send *robot* :rarm_joint1 :joint-angle 30)
(send *robot* :rarm :inverse-kinematics search-tag-pose)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(rarm-search-servo-tag "rpov-rarm_elbow_p_servo")

(setq stamp (ros::time-now))
(when (send *tfl* :wait-for-transform "dual_arm_base" "rpov-rarm_elbow_p_servo" stamp 5)
  (setq *base-to-rarm-elbow-p-servo* (send *tfl* :lookup-transform "dual_arm_base" "rpov-rarm_elbow_p_servo" stamp)))

;;<TODO recognize which cable is missing>
(rarm-grasp-cable "rpov-rarm_elbow_p_servo" "right")
