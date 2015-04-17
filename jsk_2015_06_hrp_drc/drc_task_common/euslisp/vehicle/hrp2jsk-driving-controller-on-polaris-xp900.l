#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/robot-driving-controller.l")
(load "package://drc_task_common/euslisp/vehicle/hrp2jsk-driving-motion-on-polaris-xp900.l")

(defclass hrp2jsk-driving-controller-on-polaris-xp900
  :super robot-driving-controller
  :slots (current-move-mm)
  )

(defmethod hrp2jsk-driving-controller-on-polaris-xp900
  (:init
   (&key ((:motion tmp-motion) nil)
         ((:real tmp-real) nil) ((:rate tmp-ros-rate) 10.0) (roseus-init t)
         ((:max-turn-angle tmp-max-turn-angle) 480.0)
         ((:max-turn-angle-diff tmp-max-turn-angle-diff) 180.0)
         (approach-handle nil) (approach-pedal nil)
         )
   (when roseus-init
     (ros::roseus "hrp2jsknt_driving_interface_on_polaris_xp900")
     (ros::rate tmp-ros-rate)
     )
   (unless tmp-motion
     (setq tmp-motion (instance hrp2jsk-driving-motion-on-polaris-xp900 :init :real tmp-real
                                :grasp-arm :rarm :grasp-point :handle-crank))
     )
   (send-super :init :motion tmp-motion
               :real tmp-real :rate tmp-ros-rate :roseus-init nil
               :approach-handle approach-handle :approach-pedal approach-pedal
               :max-turn-angle tmp-max-turn-angle :max-turn-angle-diff tmp-max-turn-angle-diff
               )
   (send self :step-accel-command -25.0)
   (send self :release-accel-command 1.0)
   (setq current-move-mm 0.0)
   )
  (:accel-cmd
   (cmd)
   (let ((cmd-mm (send self :get-pedal-angle-from-cmd cmd release-accel-command step-accel-command)))
     (warn "accel ~A[cmd] -> ~A [mm]~%" cmd cmd-mm)
     (send self :step-accel cmd-mm)
     (setq current-move-mm (+ current-move-mm cmd-mm))
     )
   )
  (:get-pedal-angle-from-cmd
   (data min-pedal-angle max-pedal-angle) ;; data will be 0.0 from 1.0
   (let (target-move-mm command-move-mm)
     (setq target-move-mm (+ min-pedal-angle (* data (- max-pedal-angle min-pedal-angle)))) ;; valiable name is pedal angle but this value means min/max move-mm
     (setq command-move-mm (- target-move-mm current-move-mm))
     command-move-mm
     )
   )
  (:step-brake
   (command)
   (warn "Real Vehicle Mode: Brake Disabled~%")
   (send self :step-accel 0.0) ;; release accel forcely
   )
  )

;; (setq *controller* (instance hrp2jsk-driving-controller-on-polaris-xp900 :init :real nil))
