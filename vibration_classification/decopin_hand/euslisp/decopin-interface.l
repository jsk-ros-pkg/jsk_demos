;; roslaunch decopin_hand dynamixel_controllers.launch  ;; Start communication with dynamixels
;; (decopin-init)                                 ;; create *robot* and *ri*

(require "package://pr2eus/robot-interface.l")
(load "package://decopin_hand/euslisp/decopin.l")

(defclass decopin-interface
  :super robot-interface
  :slots ()
  )

(defmethod decopin-interface
  (:init
   (&rest args)
   (prog1 (send-super* :init :robot decopin :base-controller-action-name nil :joint-states-topic "dynamixel_workbench/joint_states" args)
     (send self :add-controller :fullbody-controller)
     ))
  (:default-controller
   ()
   (append
    (send self :fullbody-controller))
   )
  (:fullbody-controller
   ()
   (list
    (list
     (cons :controller-action "dynamixel_workbench/follow_joint_trajectory_action")
     (cons :controller-state "dynamixel_workbench/follow_joint_trajectory_action/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (send-all (send robot :joint-list) :name))))
   )
  (:angle-vector
   (av &optional (tm nil) (ctype controller-type) (start-time 0) &key (min-time 0.01) &rest args)
   (send-super* :angle-vector av tm ctype start-time :min-time min-time args)
   )
  )

(defun decopin-init ()
  (unless (boundp '*robot*)
    (setq *robot* (instance decopin :init)))
  (unless (boundp '*ri*)
    (setq *ri* (instance decopin-interface :init))))

;; set the robot reset-pose before calling this function
(defun set-reset-pose ()
  ;; Sometimes this function does not return if you use :wait-until-update t
  ;; (send *robot* :set-reset-pose (send *ri* :state :potentio-vector :wait-until-update t)))
  (send *robot* :set-reset-pose (send *ri* :state :potentio-vector)))
