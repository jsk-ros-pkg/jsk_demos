#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::load-ros-manifest "sensor_msgs")

(ros::roseus "repair-robot")

(ros::advertise "/panda_order" std_msgs::String 1)

;(setq joint_state (one-shot-subscribe "/puppet_joint_states" sensor_msgs::jointstate))

;;make list of timeout-servo
(defun make-timeout-servo-list ()
    (let (timeout-joint-list)
      (setq timeout-joint-list '())
      (setq count (- (length (send joint_state :name)) 1))
      (while (> count -1)
	(if (< (elt (send joint_state :effort) count) 0.1)
	  (setq timeout-joint-list (cons (elt (send joint_state :name) count) timeout-joint-list)))
	(setq count (- count 1)))
      (print timeout-joint-list)))

(dual_panda-init)

(objects (list *robot*))

;setting
(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)

(send *robot* :rarm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(444 -96.5 1301.6) :rpy (float-vector 0.0 1.57 -1.57))) :translation-axis t :rotation-axis t)
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(432.17 224.157 1400) :rpy (float-vector -1.57 0.84 1.60))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm)


;rotate KXR's rarm by himself

;grasp
(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(433 170.8 1340.32) :rpy (float-vector -1.57 0.84 1.60))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :larm)
(unix::sleep 1)

(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(433 185.8 1310.22) :rpy (float-vector -1.533 0.56 1.63))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *robot* :larm :inverse-kinematics (send (send *robot* :copy-worldcoords) :transform (make-coords :pos #f(433 185.8 1280.22) :rpy (float-vector -1.533 0.0 1.63))) :translation-axis t :rotation-axis t)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :larm)

(send *robot* :larm :move-end-pos #f(-100 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;;move KXR's rarm by himlelf
