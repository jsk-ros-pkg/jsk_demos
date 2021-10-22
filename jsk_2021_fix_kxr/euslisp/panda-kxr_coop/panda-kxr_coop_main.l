#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::load-ros-manifest "sensor_msgs")

(ros::roseus "repair-robot")

(setq joint_state (one-shot-subscribe "/puppet_joint_states" sensor_msgs::jointstate))

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

  




;;(ros::rate 10)

;(sensor_msgs::jointstate 1)
