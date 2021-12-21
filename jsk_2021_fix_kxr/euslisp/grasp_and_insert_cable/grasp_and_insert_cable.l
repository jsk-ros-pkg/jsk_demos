#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")
(require "package://jsk_2021_fix_kxr/euslisp/panda_commons/parameters.l")
(require "package://jsk_2021_fix_kxr/euslisp/panda_commons/utils.l")

(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::load-ros-manifest "franka_msgs")
(ros::load-ros-manifest "apriltag_ros")

(ros::roseus "grasp_and_insert_cable")

(dual_panda-init)

(objects (list *robot*))

(send *ri* :set-joint-pd-gain "larm_joint1" 1000 30)
(send *ri* :set-joint-pd-gain "larm_joint2" 1000 30)
(send *ri* :set-joint-pd-gain "larm_joint3" 1000 30)
(send *ri* :set-joint-pd-gain "larm_joint4" 1000 30)
(send *ri* :set-joint-pd-gain "larm_joint5" 500 15)
(send *ri* :set-joint-pd-gain "larm_joint6" 300 15)
(send *ri* :set-joint-pd-gain "larm_joint7" 100 10)
(send *ri* :set-joint-pd-gain "rarm_joint1" 1000 30)
(send *ri* :set-joint-pd-gain "rarm_joint2" 1000 30)
(send *ri* :set-joint-pd-gain "rarm_joint3" 1000 30)
(send *ri* :set-joint-pd-gain "rarm_joint4" 1000 30)
(send *ri* :set-joint-pd-gain "rarm_joint5" 500 15)
(send *ri* :set-joint-pd-gain "rarm_joint6" 300 15)
(send *ri* :set-joint-pd-gain "rarm_joint7" 100 10)
(send *robot* :angle-vector (send *ri* :state :potentio-vector))

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
  (setq *base_to_rarm_elbow_p_servo* (send *tfl* :lookup-transform "dual_arm_base" "rpov-rarm_elbow_p_servo" stamp)))

;;<TODO recognize which cable is missing>
(rarm-grasp-cable "rpov-rarm_elbow_p_servo" "right")

(rarm-insert-cable "rpov-rarm_elbow_p_servo" "right")
(rarm-insert-cable-test "rpov-rarm_elbow_p_servo" "right")

(setq *servo-to-slide-insert-right-cable-pose*
      (make-cascoords
       :pos #f(33.3 -13.281 37.626)
       :rpy #f(-1.57 1.57 -1.57)))
