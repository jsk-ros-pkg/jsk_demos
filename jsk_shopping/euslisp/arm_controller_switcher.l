#!/usr/bin/env roseus

;; mainly referred from
;; fetch_robotics/fetch_robots fetch_bringup/scripts/controller_reset.py
;; https://github.com/fetchrobotics/fetch_robots/blob/d2923e06c9a84e70abb0ac9b89aefd6dfc7b9c2e/fetch_bringup/scripts/controller_reset.py NOQA

(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "actionlib_msgs")
(ros::roseus-add-msgs "control_msgs")
(ros::roseus-add-msgs "robot_controllers_msgs")

(ros::roseus "arm-controller-reset")

(defclass arm-controller-reset
    :slots (controller-action-name gripper-action-name
            controller-client gripper-client start stop))
(defmethod arm-controller-reset
  (:init ()
   (setq controller-action-name "/query_controller_states")
   (setq gripper-action-name "/gripper_controller/gripper_action")

   (ros::ros-info "Connecting to ~A" controller-action-name)
   (setq controller-client (instance ros::simple-action-client :init
                                     controller-action-name
                                     robot_controllers_msgs::QueryControllerStatesAction))
   (setq hoge (send controller-client :wait-for-server 1))
   (ros::ros-info "Connected to ~A" controller-action-name)

   (ros::ros-info "Connecting to ~A" gripper-action-name)
   (setq gripper-client (instance ros::simple-action-client :init
                                  gripper-action-name
                                  control_msgs::GripperCommandAction))
   (send gripper-client :wait-for-server 1)
   (ros::ros-info "Connected to ~A" gripper-action-name)
   (setq start (list "arm_controller/gravity_compensation"))
   (setq stop (list "arm_controller/follow_joint_trajectory"
                    "arm_with_torso_controller/follow_joint_trajectory"
                    "torso_controller/follow_joint_trajectory"
                    "head_controller/follow_joint_trajectory"
                    "head_controller/point_head")))
  (:arm-servo-off ()
    (let ((goal-servo-off nil) (state nil) (update nil))
      (setq goal-servo-off
            (instance robot_controllers_msgs::QueryControllerStatesGoal :init))
      (setq update (send goal-servo-off :updates))
      ;; start gravity compensation
      (dolist (controller start)
        (progn (setq state (instance robot_controllers_msgs::ControllerState :init))
               (send state :name controller)
               (send state :state 1) ; running
               (push state update)))
      ;; stop controllers
      (dolist (controller stop)
        (progn (setq state (instance robot_controllers_msgs::ControllerState :init))
               (send state :name controller)
               (send state :state 0) ; stopping
               (push state update)))
      (send goal-servo-off :updates update)
      (send controller-client :send-goal goal-servo-off)))
  (:gripper-servo-off ()
    ;; disable gripper torque
    (let ((goal-gripper-servo-off (instance control_msgs::GripperCommandGoal :init)))
      (send (send goal-gripper-servo-off :command) :max_effort -1.0)
      (send gripper-client :send-goal goal-gripper-servo-off))))
