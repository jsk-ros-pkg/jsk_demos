#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/staro-driving-controller.l")
(load "package://drc_task_common/euslisp/vehicle/staro-driving-motion-on-polaris-xp900.l")

(defclass staro-driving-controller-on-polaris-xp900
  :super staro-driving-controller
  )

(defmethod staro-driving-controller-on-polaris-xp900
  (:init
   (&key ((:real tmp-real) nil)
         ((:rate tmp-ros-rate) 10.0) (roseus-init t)
         ((:max-turn-angle tmp-max-turn-angle) 480.0)
         ((:max-turn-angle-diff tmp-max-turn-angle-diff) 180.0)
         (approach-handle nil) (approach-pedal nil)
         )
   (when roseus-init
     (ros::roseus "staro_driving_controller_on_polaris_xp900")
     (ros::rate tmp-ros-rate)
     )
   (send-super :init :motion (instance staro-driving-motion-on-polaris-xp900 :init :real tmp-real
                                       :grasp-arm :rarm :grasp-point :handle-crank)
               :real tmp-real :rate tmp-ros-rate :roseus-init nil
               :approach-handle approach-handle :approach-pedal approach-pedal
               :max-turn-angle tmp-max-turn-angle :max-turn-angle-diff tmp-max-turn-angle-diff
               )
   ;; set default accel angle
   (send self :step-accel-command 20)
   (send self :release-accel-command 0)
   )
  (:step-brake
   (command)
   (warn "Real Vehicle Mode: Brake Disabled~%")
   (send self :step-accel 0.0) ;; release accel forcely
   )
  )

;; (setq *controller* (instance staro-driving-controller-on-polaris-xp900 :init :real nil))
