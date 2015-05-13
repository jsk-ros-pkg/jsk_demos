#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/vehicle-executive-utils.l")

(ros::roseus-add-msgs "drc_com_common")
(ros::roseus "vehicle_fc_executive")

(setq *latest-handle-cmd* nil)

(defun ocs-message-callback (msg)
  (print msg)
  (let (tmp-msg)
    (setq tmp-msg (make-msg-from-tunnel msg :handle_cmd std_msgs::Float64))
    (ros::publish "/drive/operation/handle_cmd" tmp-msg)
    (setq tmp-msg (make-msg-from-tunnel msg :accel_cmd std_msgs::Float64))
    (ros::publish "/drive/operation/accel_cmd" tmp-msg)
    )
  )

(defun main-spin-once ()
  (let ((msg (instance drc_com_common::VehicleFC2OCSSmall :init)))
    (ros::publish "/fc_to_ocs_vehicle/input" msg)
    )
  )

;; for ueda tunnel
(ros::advertise "/fc_to_ocs_vehicle/input" drc_com_common::VehicleFC2OCSSmall)
(ros::subscribe "/fc_from_ocs_vehicle/output" drc_com_common::VehicleOCS2FCSmall #'ocs-message-callback)

;; message to driving-controller
(ros::advertise "/drive/operation/handle_cmd" std_msgs::Float64 1)
(ros::advertise "/drive/operation/accel_cmd" std_msgs::Float64 1)

;; message from driving-controller

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (main-spin-once)
  (ros::sleep))
