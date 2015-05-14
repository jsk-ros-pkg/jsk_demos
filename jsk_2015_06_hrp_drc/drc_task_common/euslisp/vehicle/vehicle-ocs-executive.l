#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/vehicle-executive-utils.l")

(ros::roseus-add-msgs "drc_com_common")
(ros::roseus "vehicle_ocs_executive")

(setq *latest-handle-cmd* nil)
(setq *latest-accel-cmd* nil)

(defun handle-cmd-callback (msg)
  (setq *latest-handle-cmd* (send msg :data))
  )
(defun accel-cmd-callback (msg)
  (setq *latest-accel-cmd* (send msg :data))
  )

(defun fc-message-callback (msg)
  (print msg)
  (let (tmp-msg)
    (setq tmp-msg (make-msg-from-tunnel msg :goal_handle_angle std_msgs::Float32))
    (ros::publish "/ocs/drive/controller/goal_handle_angle" tmp-msg)
    (setq tmp-msg (make-msg-from-tunnel msg :estimated_handle_angle std_msgs::Float32))
    (ros::publish "/ocs/drive/controller/estimated_handle_angle" tmp-msg)
    (setq tmp-msg (make-msg-from-tunnel msg :step std_msgs::Float32))
    (ros::publish "/ocs/drive/controller/step" tmp-msg)
    (setq tmp-msg (make-msg-from-tunnel msg :min_step std_msgs::Float32))
    (ros::publish "/ocs/drive/controller/min_step" tmp-msg)
    (setq tmp-msg (make-msg-from-tunnel msg :max_step std_msgs::Float32))
    (ros::publish "/ocs/drive/controller/max_step" tmp-msg)
    (setq tmp-msg (make-msg-from-tunnel msg :handle_mode std_msgs::String
                                        :convert #'convert-uint8-to-mode
                                        )
          )
    (ros::publish "/ocs/drive/controller/handle_mode" tmp-msg)
    (setq tmp-msg (make-msg-from-tunnel msg :accel_mode std_msgs::String
                                        :convert #'convert-uint8-to-mode
                                        )
          )
    (ros::publish "/ocs/drive/controller/accel_mode" tmp-msg)
    )
  )

(defun main-spin-once ()
  (let ((msg (instance drc_com_common::VehicleOCS2FCSmall :init)))
    (when *latest-handle-cmd*
      (send msg :handle_cmd *latest-handle-cmd*)
      )
    (when *latest-accel-cmd*
      (send msg :accel_cmd *latest-accel-cmd*)
      )
    (ros::publish "/ocs_to_fc_vehicle/input" msg)
    )
  )

;; for ueda tunnel
(ros::advertise "/ocs_to_fc_vehicle/input" drc_com_common::VehicleOCS2FCSmall)
(ros::subscribe "/ocs_from_fc_vehicle/output" drc_com_common::VehicleFC2OCSSmall #'fc-message-callback)

;; message from operation interface (drive/operation)
(ros::subscribe "/ocs/drive/operation/handle_cmd" std_msgs::Float64 #'handle-cmd-callback)
(ros::subscribe "/ocs/drive/operation/accel_cmd" std_msgs::Float64 #'accel-cmd-callback)

;; message to vehicle_ui
(ros::advertise "/ocs/drive/controller/goal_handle_angle" std_msgs::Float32 1)
(ros::advertise "/ocs/drive/controller/estimated_handle_angle" std_msgs::Float32 1)
(ros::advertise "/ocs/drive/controller/step" std_msgs::Float32 1)
(ros::advertise "/ocs/drive/controller/min_step" std_msgs::Float32 1)
(ros::advertise "/ocs/drive/controller/max_step" std_msgs::Float32 1)
(ros::advertise "/ocs/drive/controller/handle_mode" std_msgs::String 1)
(ros::advertise "/ocs/drive/controller/accel_mode" std_msgs::String 1)

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (main-spin-once)
  (ros::sleep))
