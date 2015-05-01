#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/staro-driving-controller.l")
(ros::roseus-add-msgs "drc_task_common")
(ros::roseus-add-msgs "std_msgs")

;; for demo

(defclass staro-recognition-controller-bridge
  :super object
  :slots (contorller ros-rate cmd-rate
                     cmd-start-time-handle cmd-start-time-pedal
                     turn-handle-flag step-accel-flag
                     handle-angle-offset rfsensor-value
                     )
  )

(defmethod staro-recognition-controller-bridge
  (:init
   (&key ((:controller tmp-controller) nil)
         (rate 10) ((:cmd-rate tmp-cmd-rate) 1.0) (roseus-init t))
   (when roseus-init
     (ros::roseus "staro_recognition_controller_bridge")
     )
   (unless tmp-controller
     (setq tmp-controller (instance staro-driving-controller :init
                                    :real t :roseus-init nil))
     )
   (setq *controller* controller)
   (setq controller tmp-controller)
   (setq ros-rate rate
         cmd-rate tmp-cmd-rate
         cmd-start-time-handle 0.0
         cmd-start-time-pedal 0.0          
         turn-handle-flag nil
         step-accel-flag nil
         handle-angle-offset 0.0
         )
   (ros::subscribe "/hand_wheel/local_planner/raw_cmd_angle"
                   std_msgs::Float64
                   #'send self :handle-feedback-callback)
   (ros::subscribe "/offset_estimation/relative_steering_angle"
                   std_msgs::Float64
                   #'send self :handle-diff-callback)
   (ros::subscribe "/drive/gas_pedal/cmd_angle"
                   std_msgs::Float64
                   #'send self :pedal-feedback-callback)
   (ros::subscribe "/drive/operation/flag/handle"
                   std_msgs::Bool
                   #'send self :operation-flag-handle-callback)
   (ros::subscribe "/drive/operation/flag/pedal"
                   std_msgs::Bool
                   #'send self :operation-flag-pedal-callback)
   (ros::subscribe "/obstacle_detection/stop_real_robot_cmd"
                   std_msgs::Bool
                   #'send self :obstacle-detection-callback)
   (ros::advertise "/obstacle_detection/stop_real_robot_cmd"
                   std_msgs::Bool 1)
   (ros::rate ros-rate)
   )
  
  (:handle-feedback-callback
   (msg)
   (let ((data (send msg :data)))
     (warn "received handle cmd: raw_cmd: ~A offset: ~A~%" data handle-angle-offset)
     (let* ((cmd-time (send (send (ros::time) :now) :to-nsec))
            (erapsed-time-sec (/ (- cmd-time cmd-start-time-handle) 1000000000))
            (offseted-turn-angle (+ data handle-angle-offset))
            )
       (when (and (send driving-interface :grasped-flag)
                  turn-handle-flag (> erapsed-time-sec (/ 1.0 cmd-rate)))
         (setq cmd-start-time-handle cmd-time)
         (send driving-interface :turn offseted-turn-angle :rad t :supress-diff-max t)
         )
       )
     )
   )

  (:handle-diff-callback
   (msg)
   (let ((data (send msg :data)))
     (warn "received handle-diff : ~A~%" data)
     (setq handle-angle-offset data)
     )
   )
  
  (:pedal-feedback-callback
   (msg)
   (let ((data (send msg :data)))
     (warn "received pedal cmd: ~A~%" data)
     (let* ((cmd-time (send (send (ros::time) :now) :to-nsec))
            (erapsed-time-sec (/ (- cmd-time cmd-start-time-pedal) 1000000000))
            )
       (when (and step-accel-flag (> erapsed-time-sec (/ 1.0 cmd-rate)))
         (setq cmd-start-time-pedal cmd-time)
         (send driving-interface :accel-cmd data)
         )
       )
     )
   )

  (:operation-flag-handle-callback
   (msg)
   (let ((data (send msg :data)))
     (if data
         (send self :turn-handle-flag nil)
       (send self :turn-handle-flag t)
       )
     )
   )

  (:operation-flag-pedal-callback
   (msg)
   (let ((data (send msg :data)))
     (if data
         (send self :step-accel-flag nil)
       (send self :step-accel-flag t)
       )
     )
   )

  (:obstacle-detection-callback
   (msg)
   (let ((data (send msg :data)))
     (when data
       (warn "obstacle detection called~%")
       (send driving-interface :brake-cmd 1.0)
       (send self :step-accel-flag nil) ;; disable pedal feedback control
       (send driving-interface :accel-flag nil) ;; disable handle_cotnroller pedaling
       )
     )
   )
  
  (:execute
   (&optional (rate ros-rate))
   (ros::rate rate)
   (do-until-key
    (ros::spin-once)
    (ros::sleep)
    )
   )

  (:cmd-rate (&optional (rate nil)) (when rate (setq cmd-rate rate)) rate)

  (:turn-handle-flag
   (&optional (val 0))
   (unless (or (equal val 0) (equal turn-handle-flag val))
     (warn "set turn-handle-flag ~A~%" val)
     (setq turn-handle-flag val)
     (when turn-handle-flag
       (send self :handle-angle-offset 0.0) ;; reset offset
       )
     )
   turn-handle-flag
   )
  (:step-accel-flag
   (&optional (val 0))
   (unless (or (equal val 0) (equal step-accel-flag val))
     (warn "set step-accel-flag ~A~%" val)
     (setq step-accel-flag val)
     (when step-accel-flag
       (send self :disable-obstacle-detection) ;; reset obstacle detection flag
       )
     )
   step-accel-flag
   )

  (:max-turn-angle-diff
   (&optional (angle nil))
   (when angle (setq max-turn-angle-diff angle))
   max-turn-angle-diff
   )
  (:handle-angle-offset
   (&optional (val nil))
   (when val (setq handle-angle-offset val))
   handle-angle-offset
   )
  (:disable-obstacle-detection
   (&optional (val nil))
   (let ((pub-msg (instance std_msgs::Bool :init)))
     (send pub-msg :data nil)
     (ros::publish "/obstacle_detection/stop_real_robot_cmd" pub-msg)
     (send driving-interface :accel-flag t) ;; enable handle_controller pedaling
     )
   )
  )

;; (setq *bridge* (instance staro-recognition-controller-bridge :init :real nil))
