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
               )
   (send self :step-accel-command -1.0)
   (send self :release-accel-command 25.0)
   (setq current-move-mm 0.0)
   )
  (:accel-cmd
   (cmd)
   (let ((cmd-mm (send self :get-pedal-motion-from-cmd cmd release-accel-command step-accel-command)))
     (warn "accel ~A[cmd] -> ~A [mm]~%" cmd cmd-mm)
     (send self :step-accel cmd-mm)
     (setq current-move-mm (+ current-move-mm cmd-mm))
     (send self :publish-current-step current-move-mm)
     current-move-mm ;; return total state of accel-command
     )
   )
  (:get-pedal-motion-from-cmd
   (data min-pedal-command max-pedal-command) ;; data will be 0.0 from 1.0
   (let (target-move-mm command-move-mm)
     (setq target-move-mm (+ min-pedal-command (* data (- max-pedal-command min-pedal-command))))
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
