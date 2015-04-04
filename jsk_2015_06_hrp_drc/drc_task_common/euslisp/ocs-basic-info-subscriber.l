#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/euslisp/robot-util.l")
(ros::roseus-add-msgs "drc_com_common")
(ros::roseus-add-msgs "tf2_msgs")
(ros::roseus-add-srvs "dynamic_tf_publisher")

(defclass ocs-basic-info-publisher
  :super propertied-object
  :slots (latest-joint-states))

(defmethod ocs-basic-info-publisher
  (:init ()
    self)
  (:handle-angles (msg)
    (if *use-angle-vector-decompressor*
        (let ((pubmsg (instance drc_com_common::HumanoidJointAngles :init)))
          (send pubmsg :angles (send msg :angles))
          (send pubmsg :effort (send msg :effort))
          (ros::publish "/ocs/joint_states_compressed" pubmsg))
      (let ((decompressed-effort (mapcar #'(lambda (compressed-value j)
                                             (/ (* compressed-value (send j :max-joint-torque)) 255.0))
                                         (subseq (coerce (send msg :effort) cons)
                                                 0 (length (send *robot* :angle-vector)))
                                         (send *robot* :joint-list))))
        (let ((joint-states (instance sensor_msgs::JointState :init)))
          (send joint-states :header :stamp (ros::time-now))
          (send joint-states :effort decompressed-effort)
          (send joint-states :position (subseq (coerce (send msg :angles) cons)
                0 (length (send *robot* :joint-list))))
          (send joint-states :velocity (instantiate float-vector (length (send *robot* :joint-list))))
          (send joint-states :name (send-all (send *robot* :joint-list) :name))
          (ros::publish "/ocs/joint_states_decompressed" joint-states)))))
  (:callback (msg)
    ;; angle
    (send self :handle-angles msg)
    ;; tf
    (let ((coords (float-vector-to-coords (send msg :odom_on_ground))))
      ;; coords is a transformation from odom_on_ground to BODY
      (let ((target-transform (ros::coords->tf-transform-stamped coords "BODY" "odom_on_ground")))
        (let ((req (instance dynamic_tf_publisher::SetDynamicTfRequest :init)))
          (send req :freq 20)
          (send req :cur_tf target-transform)
          (ros::service-call "/ocs/set_dynamic_tf" req)
          )))
    (let ((coords (float-vector-to-coords (send msg :ground))))
      ;; coords is a transformation from odom_on_ground to BODY
      (let ((target-transform (ros::coords->tf-transform-stamped coords "BODY" "ground")))
        (let ((req (instance dynamic_tf_publisher::SetDynamicTfRequest :init)))
          (send req :freq 20)
          (send req :cur_tf target-transform)
          (ros::service-call "/ocs/set_dynamic_tf" req)
          )))
    (let ((coords (float-vector-to-coords (send msg :odom))))
      ;; coords is a transformation from odom_on_odom to BODY
      (let ((target-transform (ros::coords->tf-transform-stamped coords "BODY" "odom")))
        (let ((req (instance dynamic_tf_publisher::SetDynamicTfRequest :init)))
          (send req :freq 20)
          (send req :cur_tf target-transform)
          (ros::service-call "/ocs/set_dynamic_tf" req)
          )))
    (let ((pubmsg (instance std_msgs::UInt8 :init)))
      (send pubmsg :data (send msg :robot_state))
      (ros::publish "/ocs/robot_status" pubmsg))
    )
  )

(ros::roseus "ocs_basic_info_publisher")
(setq *use-angle-vector-decompressor* (ros::get-param "~use_angle_vector_decompressor" nil))
(ros::wait-for-service "/ocs/set_dynamic_tf")
(setq *tf* (instance ros::transform-listener :init))
(setq *app* (instance ocs-basic-info-publisher :init))
(ros::advertise "/ocs/tf_odom_on_ground" tf2_msgs::TFMessage)
(if *use-angle-vector-decompressor*
    (ros::advertise "/ocs/joint_states_compressed" drc_com_common::HumanoidJointAngles)
  (progn
    (load (robot-file (unix::getenv "ROBOT")))
    (setq *robot* (make-robot-model-from-name (unix::getenv "ROBOT")))
    (ros::advertise "/ocs/joint_states_decompressed" sensor_msgs::JointState)))
(ros::advertise "/ocs/robot_status" std_msgs::UInt8)
(ros::subscribe "/ocs_from_fc_basic_low_speed/output"
  drc_com_common::FC2OCSBasicInfoSmall #'send *app* :callback)
;; for debug
;; (ros::subscribe "/fc_to_ocs_basic_low_speed/input"
;;   drc_com_common::FC2OCSBasicInfoSmall #'send *app* :callback)
(ros::rate 10)
(ros::spin)
