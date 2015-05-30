
;; (let* ((model2real (if (functionp 'model2real) (send 'model2real :func))))
;;   (require "package://drc_task_common/euslisp/robot-util.l")
;;   (if model2real
;;       (send 'model2real :set-val 'function model2real)))

(ros::load-ros-manifest "drc_task_common")

(require "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(defclass ocs-robot-interface
  :super rtm-ros-robot-interface
  :slots ()
  )

(defmethod ocs-robot-interface
  (:init
   (&rest
    args
    &key
    ((:robot r)
     (let* ((rn (unix:getenv "ROBOT"))
            (init (read-from-string (format nil "~A-robot" rn))))
       (cond
        ((probe-file (ros::resolve-ros-path (format nil "package://hrpsys_ros_bridge_tutorials/models/~A.l"
                                                    (string-downcase rn))))
         (require (format nil "package://hrpsys_ros_bridge_tutorials/models/~A.l"
                          (string-downcase rn)))
         (eval init)
         )
        (t (throw :robot-name-not-resolved nil)))))
    ((:groupname nh) "ocs_node_hanlder")
    &allow-other-keys)
   (ros::roseus "ocs_robot_interface")
   (setq robot (cond ((derivedp r metaclass) (instance r :init))
                     (t r)))
   ;;
   (setq groupname nh)
   (ros::create-nodehandle groupname)
   ;;
   (ros::subscribe "/ocs/joint_states_decompressed"
                   sensor_msgs::JointState
                   #'send self :ocs-joint-state-callback :groupname groupname)
   (ros::subscribe "/ocs/imu"
                   sensor_msgs::Imu
                   #'send self :ocs-imu-callback :groupname groupname)
   (ros::subscribe "/ocs/off_lfsensor"
                   geometry_msgs::WrenchStamped
                   #'send self :ocs-force-sensor-callback :lfsensor
                   :groupname groupname)
   (ros::subscribe "/ocs/off_rfsensor"
                   geometry_msgs::WrenchStamped
                   #'send self :ocs-force-sensor-callback :rfsensor
                   :groupname groupname)
   (ros::subscribe "/ocs/off_lhsensor"
                   geometry_msgs::WrenchStamped
                   #'send self :ocs-force-sensor-callback :lhsensor
                   :groupname groupname)
   (ros::subscribe "/ocs/off_rhsensor"
                   geometry_msgs::WrenchStamped
                   #'send self :ocs-force-sensor-callback :rhsensor
                   :groupname groupname)
   (ros::subscribe "/ocs/motor_states"
                   hrpsys_ros_bridge::MotorStates
                   #'send self :ocs-motor-states-callback
                   :groupname groupname)
   ;;
   (ros::advertise "/ocs_to_fc_low_speed/input" drc_com_common::OCS2FCSmall)
   ;;
   self)
  (:ocs-init (&rest args) (send* self :init args))
  (:angle-vector
   (av sleep)
   (ros::service-call "/angle_vector_command_from_ocs"
                      (instance drc_task_common::AngleVectorCommandRequest :init :angle_vector (instance std_msgs::Float32MultiArray :init :data av) :interpolation_time (instance std_msgs::Uint32 :init :data (round sleep))))
   ;; (send-angle-vector-from-ocs av sleep)))
   )
  (:state
   (&rest args)
   (ros::spin-once groupname)
   (if args
       (cond
        ((eq (car args) :angle-vector) (send robot :angle-vector))
        (t (eval (append (list 'send 'self) args)))))
   )
  ;;
  (:get-limb-ocs-param-from-keyword
   (limb)
   (case limb
     (:rarm drc_com_common::OCS2FCSmall::*RARM*)
     (:larm drc_com_common::OCS2FCSmall::*LARM*)
     (:rleg drc_com_common::OCS2FCSmall::*RLEG*)
     (:lleg drc_com_common::OCS2FCSmall::*LLEG*)
     (t drc_com_common::OCS2FCSmall::*UNDECIDED*)))
  (:start-grasp
   (limb)
   (ros::publish "/ocs_to_fc_low_speed/input"
    (instance  drc_com_common::OCS2FCSmall :init
               :type drc_com_common::OCS2FCSmall::*START_GRASP*
               :arm_side (send self :get-limb-ocs-param-from-keyword limb))))
  (:stop-grasp
   (limb)
   (ros::publish "/ocs_to_fc_low_speed/input"
    (instance  drc_com_common::OCS2FCSmall :init
               :type drc_com_common::OCS2FCSmall::*STOP_GRASP*
               :arm_side (send self :get-limb-ocs-param-from-keyword limb))))
  (:start-impedance
   (limb &rest args &key m-p d-p k-p m-r d-r k-r)
   ;; (if args (format t " :start-impedance neglect impedance params, use :start-impedance-with-param~%"))
   (let* ((param? (and (numberp m-p) (numberp d-p) (numberp k-p)
                       (numberp m-r) (numberp d-r) (numberp k-r)))
          (msg
           (instance drc_com_common::OCS2FCSmall :init
                     :type (if param?
                               drc_com_common::OCS2FCSmall::*START_IMP_WITH_PARAM*
                             drc_com_common::OCS2FCSmall::*START_IMP*)
                     :arm_side (send self :get-limb-ocs-param-from-keyword limb))))
     (if param?
         (let* ((float_params (send msg :float_params)))
           (setf (aref float_params 0) m-p)
           (setf (aref float_params 1) d-p)
           (setf (aref float_params 2) k-p)
           (setf (aref float_params 3) m-r)
           (setf (aref float_params 4) d-r)
           (setf (aref float_params 5) k-r)))
     (ros::publish "/ocs_to_fc_low_speed/input" msg)))
  (:stop-impedance
   (limb &rest args)
   (if args (format t " :start-impedance neglect impedance params, use :start-impedance-with-param~%"))
   (ros::publish "/ocs_to_fc_low_speed/input"
    (instance  drc_com_common::OCS2FCSmall :init
               :type drc_com_common::OCS2FCSmall::*STOP_IMP*
               :arm_side (send self :get-limb-ocs-param-from-keyword limb))))
  ;;
  (:start-st
   (&rest args)
   (ros::publish "/ocs_to_fc_low_speed/input"
    (instance  drc_com_common::OCS2FCSmall :init
               :type drc_com_common::OCS2FCSmall::*HRPSYS_START_ST*)))
  ;; (:stop-st
  ;;  (&rest args)
  ;;  (ros::publish "/ocs_to_fc_low_speed/input"
  ;;   (instance  drc_com_common::OCS2FCSmall :init
  ;;              :type drc_com_common::OCS2FCSmall::*HRPSYS_STOP_ST*)))
  (:start-auto-balancer
   (&rest args)
   (ros::publish "/ocs_to_fc_low_speed/input"
    (instance  drc_com_common::OCS2FCSmall :init
               :type drc_com_common::OCS2FCSmall::*HRPSYS_START_ABC*)))
  ;; (:stop-auto-balancer
  ;;  (&rest args)
  ;;  (ros::publish "/ocs_to_fc_low_speed/input"
  ;;   (instance  drc_com_common::OCS2FCSmall :init
  ;;              :type drc_com_common::OCS2FCSmall::*HRPSYS_STOP_ABC*)))
  ;; callbacks
  (:ocs-joint-state-callback
   (msg)
   (send self :put :joint-state msg)
   (map cons
        #'(lambda (name pos effort)
            (let* ((j (send robot :joint name)))
              (cond
               (j
                (send j :joint-angle (rad2deg pos))
                (send j :joint-torque effort))
               (t (format t "invalid joint name~A~%" name)))))
        (send msg :name) (send msg :position) (send msg :effort))
   )
  (:ocs-imu-callback
   (msg)
   (send self :put :imu msg)
   (send robot :newcoords
         (make-coords
          :rot
          (quaternion2matrix
           (map float-vector
                #'(lambda (k) (send msg :orientation k))
                '(:w :x :y :z)))))
   )
  (:ocs-force-sensor-callback
   (topicname msg) (send self :put topicname msg))
  (:ocs-motor-states-callback
   (msg)
   (send self :put :motor-states msg)
   (map cons
        #'(lambda (name temp)
            (let* ((j (send robot :joint name)))
              (cond
               (j (send j :put :temperature temp))
               (t (format t "invalid joint name~A~%" name)))))
        (send msg :name) (send msg :temperature)))
  ;;
  (:tmp-force-moment-vector-for-limb
   (f/m fsensor-name &rest args)
   (let ((key-name (if (stringp fsensor-name) (read-from-string (format nil ":~A" fsensor-name))
                     fsensor-name)))
     (cond
      ((send self :get key-name)
       (map float-vector
            #'(lambda (k)
                (send (send (send self :get key-name) :wrench)
                      (if (eq f/m :force) :force :torque)
                      k))
            '(:x :y :z)))
      (t (format t ":tmp-force-moment-vector-for-limb invalid fsensor=~A~%" fsensor-name)
         (float-vector 0 0 0)))))
  )

#|

(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(require "ocs-ri.lisp")
(setq *ri* (instance rtm-ros-robot-interface :init :robot hrp2jsknts-robot))
(objects (list (setq *robot* (copy-object (send *ri* :get-val 'robot)))))
