#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/util.l")
(load "package://drc_task_common/euslisp/robot-util.l")

(load (robot-file (unix::getenv "ROBOT")))
(setq *robot* (make-robot-model-from-name (unix::getenv "ROBOT")))

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "drc_com_common")
(ros::roseus-add-msgs "hrpsys_ros_bridge")
(ros::roseus-add-msgs "geometry_msgs")

(defun absmin (a b)
  (if (< (abs a) (abs b))
      a
    b)
  )

(defclass fc-basic-info-publisher
  :super propertied-object
  :slots (latest-compressed-joint-states
          latest-joint-state
          latest-imu
          robot-state-value
          latest-lfsensor-value
          latest-rfsensor-value
          latest-lhsensor-value
          latest-rhsensor-value
          latest-motor-states))

(defmethod fc-basic-info-publisher
  (:init ()
    (setq robot-state-value 0)
    self)
  (:joint-states-compressed-callback (msg)
    (setq latest-compressed-joint-states msg))
  (:joint-states-callback (msg)
    (setq latest-joint-state msg))
  (:robot-state-callback (msg)
    (setq robot-state-value (send msg :data)))
  (:motor-states-callback (msg)
    (setq latest-motor-states msg))
  (:lfsensor-callback (msg)
    (setq latest-lfsensor-value (send msg :wrench)))
  (:rfsensor-callback (msg)
    (setq latest-rfsensor-value (send msg :wrench)))
  (:lhsensor-callback (msg)
    (setq latest-lhsensor-value (send msg :wrench)))
  (:rhsensor-callback (msg)
    (setq latest-rhsensor-value (send msg :wrench)))
  (:arm-forces ()
    (when (and latest-lhsensor-value latest-rhsensor-value)
      (let ((left (send self :arm-force-vector latest-lhsensor-value))
            (right (send self :arm-force-vector latest-rhsensor-value)))
        (let ((ret (instantiate integer-vector 12)))
          (dotimes (i 6)
            (setf (elt ret i) (elt left i))
            (setf (elt ret (+ 6 i)) (elt right i)))
          ret))))
  (:arm-force-vector (wrench)
     (let ((vec (instantiate integer-vector 6)))
       ;; force
       (setf (elt vec 0) (* 128 (/ (absmin *arm-force-max* (send wrench :force :x))
                                   *arm-force-max*)))
       (setf (elt vec 1) (* 128 (/ (absmin *arm-force-max* (send wrench :force :y))
                                   *arm-force-max*)))
       (setf (elt vec 2) (* 128 (/ (absmin *arm-force-max* (send wrench :force :z))
                                   *arm-force-max*)))
       ;; torque
       (setf (elt vec 3) (* 128 (/ (absmin *arm-torque-max* (send wrench :torque :x))
                                   *arm-torque-max*)))
       (setf (elt vec 4) (* 128 (/ (absmin *arm-torque-max* (send wrench :torque :y))
                                   *arm-torque-max*)))
       (setf (elt vec 5) (* 128 (/ (absmin *arm-torque-max* (send wrench :torque :z))
                                   *arm-torque-max*)))
       vec))
  (:leg-forces ()
    (when (and latest-lfsensor-value latest-rfsensor-value)
      (let ((left (send self :leg-force-vector latest-lfsensor-value))
            (right (send self :leg-force-vector latest-rfsensor-value)))
        (let ((ret (instantiate integer-vector 12)))
          (dotimes (i 6)
            (setf (elt ret i) (elt left i))
            (setf (elt ret (+ 6 i)) (elt right i)))
          ret))))
  (:leg-force-vector (wrench)
     (let ((vec (instantiate integer-vector 6)))
       ;; force
       (setf (elt vec 0) (* 128 (/ (absmin *leg-force-max* (send wrench :force :x))
                                   *leg-force-max*)))
       (setf (elt vec 1) (* 128 (/ (absmin *leg-force-max* (send wrench :force :y))
                                   *leg-force-max*)))
       (setf (elt vec 2) (* 128 (/ (absmin *leg-force-max* (send wrench :force :z))
                                   *leg-force-max*)))
       ;; torque
       (setf (elt vec 3) (* 128 (/ (absmin *leg-torque-max* (send wrench :torque :x))
                                   *leg-torque-max*)))
       (setf (elt vec 4) (* 128 (/ (absmin *leg-torque-max* (send wrench :torque :y))
                                   *leg-torque-max*)))
       (setf (elt vec 5) (* 128 (/ (absmin *leg-torque-max* (send wrench :torque :z))
                                   *leg-torque-max*)))
       vec))
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
  (:imu-callback
   (msg)
   (setq latest-imu msg))
  (:report ()
    (let ((msg (instance drc_com_common::FC2OCSBasicInfoSmall :init)))
      (when latest-compressed-joint-states
        ;; (send msg :angles (send latest-compressed-joint-states :angles)))
        (send msg :effort (send latest-compressed-joint-states :effort)))
      ;; temperature from motor states
      (when latest-motor-states
        (dotimes (i (min (length (send latest-motor-states :temperature)) 
                         (length (send msg :temperature))))
          (setf (elt (send msg :temperature) i) 
                (round (elt (send latest-motor-states :temperature) i)))))
      ;; force
      (when (and latest-lfsensor-value
                 latest-rfsensor-value
                 latest-lhsensor-value
                 latest-rhsensor-value)
        (send msg :arm_forces (send self :arm-forces))
        (send msg :leg_forces (send self :leg-forces)))
      (when latest-joint-state
        (let ((av (send self :angle-vector-from-joint-states latest-joint-state)))
          (when av
            (dotimes (i (length av))
              (setf (elt (send msg :angles) i) (elt av i))))))
      (when latest-imu
        (setf (elt (send msg :imu) 0) (send latest-imu :orientation :x))
        (setf (elt (send msg :imu) 1) (send latest-imu :orientation :y))
        (setf (elt (send msg :imu) 2) (send latest-imu :orientation :z))
        (setf (elt (send msg :imu) 3) (send latest-imu :orientation :w)))
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
(setq *arm-force-max* (ros::get-param "~arm_force_max" 256.0))
(setq *arm-torque-max* (ros::get-param "~arm_torque_max" 256.0))
(setq *leg-force-max* (ros::get-param "~leg_force_max" 728.0))
(setq *leg-torque-max* (ros::get-param "~leg_torque_max" 728.0))

(ros::advertise "/fc_to_ocs_basic_low_speed/input" 
  drc_com_common::FC2OCSBasicInfoSmall)
(ros::subscribe "/joint_states_compressed" drc_com_common::HumanoidJointAngles
  #'send *app* :joint-states-compressed-callback)
(ros::subscribe "/joint_states_appended" sensor_msgs::JointState
  #'send *app* :joint-states-callback)
(ros::subscribe "/communication/robot_status" std_msgs::UInt8
  #'send *app* :robot-state-callback)
(ros::subscribe "/imu" sensor_msgs::Imu
                #'send *app* :imu-callback)
(ros::subscribe "/motor_states" hrpsys_ros_bridge::MotorStates
                #'send *app* :motor-states-callback)
(ros::subscribe "/off_lfsensor" geometry_msgs::WrenchStamped
                #'send *app* :lfsensor-callback)
(ros::subscribe "/off_rfsensor" geometry_msgs::WrenchStamped
                #'send *app* :rfsensor-callback)
(ros::subscribe "/off_lhsensor" geometry_msgs::WrenchStamped
                #'send *app* :lhsensor-callback)
(ros::subscribe "/off_rhsensor" geometry_msgs::WrenchStamped
                #'send *app* :rhsensor-callback)

(ros::rate 10)
(while (ros::ok)
  (ros::sleep)
  (ros::spin-once)
  (send *app* :report))


