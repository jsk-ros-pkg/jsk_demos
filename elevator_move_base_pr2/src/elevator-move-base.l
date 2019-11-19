#!/usr/bin/env roseus

(ros::roseus-add-srvs "nav_msgs")
(ros::roseus-add-srvs "std_srvs")

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

(defun call-service (service req)
  (if (ros::wait-for-service service 0)
      (ros::service-call service req)
      nil))

(defun relay-make-plan (req)
  (call-service "/move_base_node/make_plan" req))

(defun relay-clear-costmaps (req)
  (call-service "/move_base/clear_costmaps" req))


(ros::roseus "elevator_move_base")

(initialize-env)
(ros::advertise "/elevator_move_base/goal" move_base_msgs::MoveBaseActionGoal 1)
(unix:usleep (* 100 1000))  ;; Wait for setting up publisher
(ros::subscribe "/elevator_move_base_simple/goal" geometry_msgs::PoseStamped
                #'simple-goal-cb)

(ros::advertise-service "/elevator_move_base/make_plan"
                        nav_msgs::GetPlan #'relay-make-plan)
(ros::advertise-service "/elevator_move_base/clear_costmaps"
                        std_srvs::Empty #'relay-clear-costmaps)

(setq sm (elevator-smach))
(setq s (instance state-machine-action-server :init "/elevator_move_base"
                  move_base_msgs::MoveBaseAction sm))
(send s :success-state '(:success)) ;; name of success state
(ros::ros-info "initialized elevator-move-base.l")
(ros::rate 10)
(while (ros::ok)
 (ros::spin-once)
 (send s :worker)
 (ros::sleep))
