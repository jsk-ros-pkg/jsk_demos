#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/util.l")
(ros::roseus-add-msgs "drc_com_common")
(ros::roseus-add-msgs "tf2_msgs")
(ros::roseus-add-srvs "dynamic_tf_publisher")

(defclass ocs-basic-info-publisher
  :super propertied-object
  :slots (latest-joint-states))

(defmethod ocs-basic-info-publisher
  (:init ()
    self)
  (:callback (msg)
    ;; angle
    (let ((pubmsg (instance drc_com_common::HumanoidJointAngles :init)))
      (send pubmsg :angles (send msg :angles))
      (ros::publish "/ocs/joint_states_compressed" pubmsg))
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
    )
  )

(ros::roseus "ocs_basic_info_publisher")
(ros::wait-for-service "/ocs/set_dynamic_tf")
(setq *tf* (instance ros::transform-listener :init))
(setq *app* (instance ocs-basic-info-publisher :init))
(ros::advertise "/ocs/tf_odom_on_ground" tf2_msgs::TFMessage)
(ros::advertise "/ocs/joint_states_compressed" drc_com_common::HumanoidJointAngles)
(ros::subscribe "/ocs_from_fc_basic_low_speed/output"
  drc_com_common::FC2OCSBasicInfoSmall #'send *app* :callback)
(ros::rate 10)
(ros::spin)
