#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/vehicle-executive-utils.l")

(ros::roseus-add-msgs "drc_com_common")
(ros::roseus "vehicle_ocs_executive")

(setq *latest-handle-cmd* nil)
(setq *latest-accel-cmd* nil)
(setq *latest-initialize-request* nil)
(setq *latest-grasp-request* nil)
(setq *latest-release-request* nil)
(setq *latest-correct-request* nil)
(setq *latest-reach-arm-request* nil)
(setq *latest-reach-leg-request* nil)

(defun handle-cmd-callback (msg)
  (setq *latest-handle-cmd* (send msg :data))
  )
(defun accel-cmd-callback (msg)
  (setq *latest-accel-cmd* (send msg :data))
  )

(defun initialize-service-callback
   (req)
   (let ((res (send req :response)))
     (setq *latest-initialize-request* t)
     res ;; return immediately
     )
   )

(defun grasp-service-callback
   (req)
   (let ((res (send req :response)))
     (setq *latest-grasp-request* t)
     res ;; return immediately
     )
   )

(defun release-service-callback
   (req)
   (let ((res (send req :response)))
     (setq *latest-release-request* t)
     res ;; return immediately
     )
   )

(defun correct-service-callback
   (req)
   (let ((res (send req :response)))
     (setq *latest-correct-request* t)
     res ;; return immediately
     )
   )
(defun reach-arm-service-callback
   (req)
   (let ((res (send req :response)))
     (setq *latest-reach-arm-request* t)
     res ;; return immediately
     )
   )
(defun reach-leg-service-callback
   (req)
   (let ((res (send req :response)))
     (setq *latest-reach-leg-request* t)
     res ;; return immediately
     )
   )

(defun fc-message-callback (msg)
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
    (when (and *latest-initialize-request* (send msg :initialize_response))
      (setq *latest-initialize-request* nil)
      )
    (when (and *latest-grasp-request* (send msg :grasp_response))
      (setq *latest-grasp-request* nil)
      )
    (when (and *latest-release-request* (send msg :release_response))
      (setq *latest-release-request* nil)
      )
    (when (and *latest-correct-request* (send msg :correct_response))
      (setq *latest-correct-request* nil)
      )
    (when (and *latest-reach-arm-request* (send msg :reach_arm_response))
      (setq *latest-reach-arm-request* nil)
      )
    (when (and *latest-reach-leg-request* (send msg :reach_leg_response))
      (setq *latest-reach-leg-request* nil)
      )
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
    (send msg :initialize_request *latest-initialize-request*)
    (send msg :grasp_request *latest-grasp-request*)
    (send msg :release_request *latest-release-request*)
    (send msg :correct_request *latest-correct-request*)
    (send msg :reach_arm_request *latest-reach-arm-request*)
    (send msg :reach_leg_request *latest-reach-leg-request*)
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

(ros::advertise-service "/ocs/drive/controller/initialize" std_srvs::Empty #'initialize-service-callback)
(ros::advertise-service "/ocs/drive/controller/grasp" std_srvs::Empty #'grasp-service-callback)
(ros::advertise-service "/ocs/drive/controller/release" std_srvs::Empty #'release-service-callback)
(ros::advertise-service "/ocs/drive/controller/correct" std_srvs::Empty #'correct-service-callback)
(ros::advertise-service "/ocs/drive/controller/reach_arm" std_srvs::Empty #'reach-arm-service-callback)
(ros::advertise-service "/ocs/drive/controller/reach_leg" std_srvs::Empty #'reach-leg-service-callback)

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (main-spin-once)
  (ros::sleep))
