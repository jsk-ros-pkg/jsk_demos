#!/usr/bin/env roseus


(ros::roseus "joint_states_storage")
(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "drc_task_common")
(ros::roseus-add-msgs "sensor_msgs")
(load "package://drc_task_common/euslisp/robot-util.l")
(load "package://drc_task_common/euslisp/util.l")


;(setq *queue* nil)

(defun joint-state-array-callback (msg)
  (dotimes (i (length (send msg :joint_state_array)))
    (let ((msg-single (elt (send msg :joint_state_array) i)))
        (send msg-single :header (instance std_msgs::Header :init :stamp (ros::time 0.0)))
        (ros::publish "/jsk_model_marker_interface/robot/reset_joint_states_and_root" msg-single)
        (unix::usleep (* (send msg :step_time) 1000))
      )
    )
  )


(ros::subscribe "/model_joint_state_array" drc_task_common::JointStateArray #'joint-state-array-callback 5)
(ros::advertise "/jsk_model_marker_interface/robot/reset_joint_states_and_root" sensor_msgs::JointState 1)
(ros::rate 10)
(ros::spin)
  
