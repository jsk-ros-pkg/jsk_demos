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

;; message from operation interface
(ros::subscribe "/ocs/drive/operation/handle_cmd" std_msgs::Float64 #'handle-cmd-callback)
(ros::subscribe "/ocs/drive/operation/accel_cmd" std_msgs::Float64 #'accel-cmd-callback)

;; message to vehicle_ui


(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (main-spin-once)
  (ros::sleep))
