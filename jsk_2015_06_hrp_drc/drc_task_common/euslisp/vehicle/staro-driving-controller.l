#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/staro-driving-motion.l")
(load "package://drc_task_common/euslisp/vehicle/robot-driving-controller.l")
;; (load "package://drc_task_common/euslisp/vehicle/staro-ps3joy-adjustment.l")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(defclass staro-driving-controller
  :super robot-driving-controller
  )

(defmethod staro-driving-controller
  (:init
   (&key ((:real tmp-real) nil) ((:motion tmp-motion) nil)
         ((:rate tmp-ros-rate) 10.0) (roseus-init t)
         (approach-handle nil) (approach-pedal nil)
         )
   (when roseus-init
     (ros::roseus "staro_driving_controller")
     (ros::rate tmp-ros-rate)
     )
   (unless tmp-motion
     (setq tmp-motion (instance staro-driving-motion :init :real tmp-real))
     )
   (send-super :init :real tmp-real :motion tmp-motion
               :rate tmp-ros-rate :roseus-init nil
               :approach-handle approach-handle :approach-pedal approach-pedal
               )
   )
  )

;; (setq *controller* (instance staro-driving-controller :init :real nil))

