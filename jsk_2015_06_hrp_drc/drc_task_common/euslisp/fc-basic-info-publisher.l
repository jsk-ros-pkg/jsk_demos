#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/euslisp/robot-util.l")

(load (robot-file (unix::getenv "ROBOT")))
(setq *robot* (make-robot-model-from-name (unix::getenv "ROBOT")))

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "drc_com_common")
(defclass fc-basic-info-publisher
  :super propertied-object
  :slots (latest-compressed-joint-states latest-joint-states robot-state-value))

(defmethod fc-basic-info-publisher
  (:init ()
    (setq robot-state-value 0)
    self)
  (:joint-states-compressed-callback (msg)
    (setq latest-compressed-joint-states msg))
  (:joint-states-callback (msg)
    (setq latest-joint-states msg))
  (:robot-state-callback (msg)
    (setq robot-state-value (send msg :data)))
  (:joint-value-from-name 
   (msg name)
   (dotimes (i (length (send msg :name)))
     (let ((candidate-joint-name (elt (send msg :name) i)))
       (if (string= name candidate-joint-name)
           (return-from :joint-value-from-name (elt (send msg :position) i)))))
   nil)
  (:angle-vector-from-joint-states 
   (msg)
   (let* ((joint-list (send *robot* :joint-list))
          (av (instantiate float-vector (length joint-list))))
     (dotimes (i (length joint-list))
       (let ((joint-name (send (elt joint-list i) :name)))
         (let ((v (send self :joint-value-from-name msg joint-name)))
           (if v (setf (elt av i) v)
             (progn
               (ros::ros-warn "Failed to find ~A joint value" joint-name)
               (return-from :angle-vector-from-joint-states nil))))))
     av))
  (:report ()
    (let ((msg (instance drc_com_common::FC2OCSBasicInfoSmall :init)))
      (when latest-compressed-joint-states
        ;; (send msg :angles (send latest-compressed-joint-states :angles)))
        (send msg :effort (send latest-compressed-joint-states :effort)))
      (when latest-joint-states
        (let ((av (send self :angle-vector-from-joint-states latest-joint-states)))
          (when av
            (dotimes (i (length av))
              (setf (elt (send msg :angles) i) (elt av i))))))
      ;; lookup body -> odom_on_ground
      (let ((trans (send *tf* :lookup-transform "BODY" "odom_on_ground"
                         (ros::time 0))))
        (when trans
          (send msg :odom_on_ground (coords-to-float-vector 
                                     trans 6))))
      ;; lookup body -> ground
      (let ((trans (send *tf* :lookup-transform "BODY" "ground" (ros::time 0))))
        (when trans
            (send msg :ground (coords-to-float-vector 
                               trans 6))))
      (let ((trans (send *tf* :lookup-transform "BODY" "odom" (ros::time 0))))
        (when trans
            (send msg :odom (coords-to-float-vector
                             trans 6))))
      (send msg :robot_state robot-state-value)
      (ros::publish "/fc_to_ocs_basic_low_speed/input" msg)))
  )

(ros::roseus "fc_basic_info_publisher")
(setq *tf* (instance ros::transform-listener :init))
(setq *app* (instance fc-basic-info-publisher :init))
(ros::advertise "/fc_to_ocs_basic_low_speed/input" 
  drc_com_common::FC2OCSBasicInfoSmall)
(ros::subscribe "/joint_states_compressed" drc_com_common::HumanoidJointAngles
  #'send *app* :joint-states-compressed-callback)
(ros::subscribe "/joint_states_appended" sensor_msgs::JointState
  #'send *app* :joint-states-callback)
(ros::subscribe "/communication/robot_status" std_msgs::UInt8
  #'send *app* :robot-state-callback)


(ros::rate 10)
(while (ros::ok)
  (ros::sleep)
  (ros::spin-once)
  (send *app* :report))


