#!/usr/bin/env roseus

(ros::roseus "elevator_move_base")

(load "package://elevator_move_base_pr2/src/elevator-move-base-main.l")

(defun simple-goal-cb (msg)
  (let ((pub-msg (instance move_base_msgs::MoveBaseActionGoal :init))
	(stamp (ros::time-now)))
    (send pub-msg :header :stamp stamp)
    (send pub-msg :goal_id :stamp stamp)
    (send pub-msg :goal_id :id (format nil "~A" (send stamp :sec-nsec)))
    (send pub-msg :goal :target_pose msg)
    (clear-costmap)
    (ros::publish "/elevator_move_base/goal" pub-msg)
    ))

;;
(initialize-env)
(ros::subscribe "/elevator_move_base_simple/goal" geometry_msgs::PoseStamped
		#'simple-goal-cb)
(ros::advertise "/elevator_move_base/goal" move_base_msgs::MoveBaseActionGoal 1)

(setq sm (elevator-smach))
(setq s (instance state-machine-action-server :init "/elevator_move_base"
                  move_base_msgs::MoveBaseAction sm))
(send s :success-state '(:success)) ;; name of success state
(ros::ros-info "initialized elevator-move-base.l")
(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (send s :worker)
 (ros::sleep))

