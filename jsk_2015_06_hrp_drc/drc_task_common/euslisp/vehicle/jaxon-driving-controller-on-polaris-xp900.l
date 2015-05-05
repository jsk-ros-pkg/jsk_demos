#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/robot-driving-controller.l")
(load "package://drc_task_common/euslisp/vehicle/jaxon-driving-motion-on-polaris-xp900.l")

(defclass jaxon-driving-controller-on-polaris-xp900
  :super robot-driving-controller
  )

(defmethod jaxon-driving-controller-on-polaris-xp900
  (:init
   (&key ((:real tmp-real) nil) ((:rate tmp-ros-rate) 10.0) (roseus-init t)
         (approach-handle nil) (approach-pedal nil)
         )
   (when roseus-init
     (ros::roseus "jaxon_driving_interface_on_polaris_xp900")
     (ros::rate tmp-ros-rate)
     )
   (send-super :init :motion (instance jaxon-driving-motion-on-polaris-xp900 :init :real tmp-real)
               :real tmp-real :rate tmp-ros-rate :roseus-init nil
               :approach-handle approach-handle :approach-pedal approach-pedal
               )
   (send self :accel-origin 4)
   (send self :step-accel-command 6)
   (send self :release-accel-command 0)
   ;; (send self :step-accel-command 10)
   ;; (send self :release-accel-command 4)
   )
  (:brake-cmd
   (cmd)
   (warn "Real Vehicle Mode: Brake Disabled~%")
   (send self :accel-cmd 0.0) ;; release accel forcely
   )
  )

(defun jaxon-vehicle-main
  (&key (real t))
  (robot-vehicle-main :robot-name "jaxon" :postfix "on-polaris-xp900" :real real)
  )

;; (setq *controller* (instance jaxon-driving-controller-on-polaris-xp900 :init :real nil))
