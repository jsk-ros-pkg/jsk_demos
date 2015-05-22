#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/robot-driving-controller.l")
(load "package://drc_task_common/euslisp/vehicle/hrp2jsk-driving-motion-on-polaris-xp900.l")

(defclass hrp2jsk-driving-controller-on-polaris-xp900
  :super robot-driving-controller
  )

(defmethod hrp2jsk-driving-controller-on-polaris-xp900
  (:init
   (&key ((:motion tmp-motion) nil)
         ((:real tmp-real) nil) ((:rate tmp-ros-rate) 10.0) (roseus-init t)
         )
   (when roseus-init
     (ros::roseus "hrp2jsknt_driving_interface_on_polaris_xp900")
     (ros::rate tmp-ros-rate)
     )
   (unless tmp-motion
     (setq tmp-motion (instance hrp2jsk-driving-motion-on-polaris-xp900 :init :real nil
                                :grasp-arm :rarm :grasp-point :handle-crank))
     )
   (send-super :init :motion tmp-motion
               :real tmp-real :rate tmp-ros-rate :roseus-init nil
               )
   (send self :step-accel-command -50.0)
   (send self :release-accel-command 0.0)
   ;; (send self :accel-origin 0.0)
   )
  (:initialize
   (&rest args &key (fist-offset #f(0 0 140)) (floor-offset #f(0 0 0))  &allow-other-keys)
   (send-super* :initialize
                :fist-offset fist-offset :floor-offset floor-offset
                args)
   )
  (:approach-accel
   (&rest args &key (limb :rleg) &allow-other-keys)
   (send* motion :approach-accel-pedal :limb limb args)
   (send self :accel-origin 0.0) ;; accel-origin means current total move distance and origin to next relative move distance
   (send self :controller-state :accel :running) ;; enbale accel
   )
  (:accel-cmd
   (cmd)
   (let ((cmd-mm (send self :get-pedal-motion-from-cmd cmd release-accel-command step-accel-command)))
     (warn "accel ~A[cmd] -> ~A [mm]~%" cmd cmd-mm)
     (send self :step-accel cmd-mm :use-controller t) ;; step-accel assumed to get relative distance to move
     (setq accel-origin (+ accel-origin cmd-mm))
     (send self :publish-current-step accel-origin)
     (send self :publish-step-on-flag cmd)
     accel-origin ;; return total state of accel-command
     )
   )
  (:get-pedal-motion-from-cmd
   (data min-pedal-command max-pedal-command) ;; data will be 0.0 from 1.0
   (let (target-move-mm command-move-mm)
     (setq target-move-mm (+ min-pedal-command (* data (- max-pedal-command min-pedal-command))))
     (setq command-move-mm (- target-move-mm accel-origin))
     command-move-mm ;; return relative distance to move
     )
   )
  (:brake-cmd
   (cmd)
   (warn "Real Vehicle Mode: Brake Disabled~%")
   (send self :accel-cmd 0.0) ;; release accel forcely
   )
  )

(defun hrp2jsk-vehicle-main
  (&key (real t))
  (robot-vehicle-main :robot-name "hrp2jsk" :postfix "on-polaris-xp900" :real real)
  )

;; (setq *controller* (instance hrp2jsk-driving-controller-on-polaris-xp900 :init :real nil))
