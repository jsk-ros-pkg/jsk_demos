#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/robot-driving-controller.l")
(load "package://drc_task_common/euslisp/vehicle/jaxon-driving-motion-on-polaris-xp900-toe-kick-accel.l")

(defclass jaxon-driving-controller-on-polaris-xp900
  :super robot-driving-controller
  )

(defmethod jaxon-driving-controller-on-polaris-xp900
  (:init
   (&key ((:motion tmp-motion) nil)
         ((:real tmp-real) nil) ((:rate tmp-ros-rate) 10.0) (roseus-init t)
         )
   (when roseus-init
     (ros::roseus "jaxon_driving_interface_on_polaris_xp900")
     (ros::rate tmp-ros-rate)
     )
   (unless tmp-motion
     (setq tmp-motion (instance jaxon-driving-motion-on-polaris-xp900 :init :real nil
                                :grasp-arm :larm :grasp-point :handle-crank))
     )
   (send-super :init :motion tmp-motion
               :real tmp-real :rate tmp-ros-rate :roseus-init nil
               )
   ;; (send self :accel-origin 4)
   ;; (send self :accel-origin -9.69405)
   ;; (send self :step-accel-command 6)
   ;; (send self :step-accel-command 15.0)
   ;; (send self :release-accel-command 5.0)
   ;; (send self :detach-accel-command 0.0)
   ;; (send self :step-accel-command 10)
   ;; (send self :release-accel-command 4)
   (send self :step-accel-command 50.0)
   (send self :release-accel-command 0.0)
   (send self :detach-accel-command -5.0)
   )
  (:initialize
   (&rest args &key (fist-offset #f(0 0 25)) (floor-offset #f(0 0 0))  &allow-other-keys)
   (send-super* :initialize
                :fist-offset fist-offset :floor-offset floor-offset
                args)
   )
  (:execute-handle-pose-compensation
   (&rest args &key (release-offset #f(-200 0 0)) (shoulder-y-deg 0.0) &allow-other-keys)
   (send-super* :execute-handle-pose-compensation
                :release-offset release-offset :shoulder-y-deg shoulder-y-deg
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
     (warn "accel ~A[cmd] -> ~A [mm], current ~A[mm]~%" cmd cmd-mm accel-origin)
     (when (send self :step-accel cmd-mm :use-controller t) ;; step-accel assumed to get relative distance to move
         (setq accel-origin (+ accel-origin cmd-mm))
         (send self :publish-current-step accel-origin)
         (send self :publish-step-on-flag cmd)
       )
     accel-origin ;; return total state of accel-command
     )
   )
  (:detach-accel
   (&key (use-controller t) (time 1000))
   (when (equal (send self :controller-state :accel) :initialize)
     (warn "accel is not approached~%")
     (return-from :detach-accel nil)
     )
   (let ((cmd-mm (- (send self :detach-accel-command) accel-origin)))
     (warn "detach-accel ~A [mm], current ~A[mm]~%" cmd-mm accel-origin)
     (when (send self :step-accel cmd-mm :time time :use-controller use-controller) ;; detach accel for emergency
       (send self :controller-state :accel :release)
       (setq accel-origin (+ accel-origin cmd-mm))
       )
     accel-origin
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

  ;; (:brake-cmd
  ;;  (cmd)
  ;;  (warn "Real Vehicle Mode: Brake Disabled~%")
  ;;  (send self :accel-cmd 0.0) ;; release accel forcely
  ;;  )
  )

(defun jaxon-vehicle-main
  (&key (real t))
  (robot-vehicle-main :robot-name "jaxon" :postfix "on-polaris-xp900" :real real)
  )

;; (setq *controller* (instance jaxon-driving-controller-on-polaris-xp900 :init :real nil))
