#!/usr/bin/env roseus

(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-srvs "nav_msgs")
(ros::roseus-add-srvs "std_srvs")

(require :state-machine-actionlib
         "package://roseus_smach/src/state-machine-actionlib.l")

(load "package://elevator_move_base_pr2/src/state-machine-main.l")
(load "package://elevator_move_base_pr2/src/utils.l")

(defvar *sm*)


(defun simple-goal-cb (msg)
  (let ((pub-msg (instance move_base_msgs::MoveBaseActionGoal :init))
        (stamp (ros::time-now)))
    (send pub-msg :header :stamp stamp)
    (send pub-msg :goal_id :stamp stamp)
    (send pub-msg :goal_id :id (format nil "~A" (send stamp :sec-nsec)))
    (send pub-msg :goal :target_pose msg)
    (ros::publish "/elevator_move_base/goal" pub-msg)))


(ros::roseus "elevator_move_base")

(initialize-env)
(ros::advertise "view_target" geometry_msgs::PointStamped 1)
(ros::advertise "/elevator_move_base/goal" move_base_msgs::MoveBaseActionGoal 1)
(unix:usleep (* 100 1000))  ;; Wait for setting up publisher
(ros::subscribe "/elevator_move_base_simple/goal" geometry_msgs::PoseStamped
                #'simple-goal-cb)

;; Relay services
(ros::advertise-service
 "/elevator_move_base/make_plan" nav_msgs::GetPlan
 #'(lambda (req) (call-service "/move_base_node/make_plan" req)))
(ros::advertise-service
 "/elevator_move_base/clear_costmaps" std_srvs::Empty
 #'(lambda (req) (call-service "/move_base/clear_costmaps" req)))

(setq *sm* (instance state-machine-action-server :init "/elevator_move_base"
                     move_base_msgs::MoveBaseAction
                     (elevator-move-base-state-machine)
                     '((push-arm . :larm))))
(send *sm* :success-state '(:succeed)) ;; name of success state
(ros::ros-info "initialized elevator-move-base.l")

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (send *sm* :worker)
  (ros::sleep))
