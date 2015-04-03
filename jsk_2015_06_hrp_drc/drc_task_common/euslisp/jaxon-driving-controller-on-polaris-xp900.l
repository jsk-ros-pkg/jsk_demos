#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/robot-driving-controller.l")
(load "package://drc_task_common/euslisp/jaxon-driving-motion-on-polaris-xp900.l")

(defclass jaxon-driving-controller-on-polaris-xp900
  :super robot-driving-controller
  )

(defmethod jaxon-driving-controller-on-polaris-xp900
  (:init
   (&key ((:real tmp-real) nil) ((:rate tmp-ros-rate) 10.0) (roseus-init t)
         ((:max-turn-angle tmp-max-turn-angle) 480.0)
         ((:max-turn-angle-diff tmp-max-turn-angle-diff) 180.0)
         (approach-handle nil) (approach-pedal nil)
         )
   (when roseus-init
     (ros::roseus "jaxon_driving_interface_on_polaris_xp900")
     (ros::rate tmp-ros-rate)
     )
   (send-super :init :motion (instance jaxon-driving-motion-on-polaris-xp900 :init :real tmp-real)
               :real tmp-real :rate tmp-ros-rate :roseus-init nil
               :approach-handle approach-handle :approach-pedal approach-pedal
               :max-turn-angle tmp-max-turn-angle :max-turn-angle-diff tmp-max-turn-angle-diff
               )
   )
  )

;; (setq *controller* (instance jaxon-driving-controller-on-polaris-xp900 :init :real nil))
