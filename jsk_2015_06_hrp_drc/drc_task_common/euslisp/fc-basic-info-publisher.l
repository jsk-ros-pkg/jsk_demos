#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/util.l")
(ros::roseus-add-msgs "drc_com_common")
(defclass fc-basic-info-publisher
  :super propertied-object
  :slots (latest-joint-states))
(defmethod fc-basic-info-publisher
  (:init ()
    self)
  
  (:joint-states-compressed-callback (msg)
    (setq latest-joint-states (send msg :angles)))
  (:report ()
    (let ((msg (instance drc_com_common::FC2OCSBasicInfoSmall :init)))
      (if latest-joint-states
          (send msg :angles latest-joint-states))
      ;; lookup body -> odom_on_ground
      (let ((trans (send *tf* :lookup-transform "BODY" "odom_on_ground" (ros::time 0))))
        (when trans
          (ros::ros-info ":odom_on_ground ~A" (send msg :odom_on_ground))
          (ros::ros-info "trans ~A" trans)
          (send msg :odom_on_ground (coords-to-float-vector 
                                     trans
                                     (length (send msg :odom_on_ground))))))
      ;; lookup body -> ground
      (let ((trans (send *tf* :lookup-transform "BODY" "ground" (ros::time 0))))
        (when trans
          (ros::ros-info ":ground ~A" (send msg :ground))
          (ros::ros-info "trans ~A" trans)
            (send msg :ground (coords-to-float-vector 
                               trans
                               (length (send msg :ground))))))
      (ros::publish "/fc_to_ocs_basic_low_speed/input" msg)))
  )

(ros::roseus "fc_basic_info_publisher")
(setq *tf* (instance ros::transform-listener :init))
(setq *app* (instance fc-basic-info-publisher :init))
(ros::advertise "/fc_to_ocs_basic_low_speed/input" 
  drc_com_common::FC2OCSBasicInfoSmall)
(ros::subscribe "/joint_states_compressed" drc_com_common::HumanoidJointAngles
  #'send *app* :joint-states-compressed-callback)

(ros::rate 10)
(while (ros::ok)
  (ros::sleep)
  (ros::spin-once)
  (send *app* :report))


