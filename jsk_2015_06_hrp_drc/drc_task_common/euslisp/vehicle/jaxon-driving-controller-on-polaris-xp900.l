#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/robot-driving-controller.l")
(load "package://drc_task_common/euslisp/vehicle/jaxon-driving-motion-on-polaris-xp900.l")

(defclass jaxon-driving-controller-on-polaris-xp900
  :super robot-driving-controller
  )

(defmethod jaxon-driving-controller-on-polaris-xp900
  (:init
   (&key ((:real tmp-real) nil) ((:rate tmp-ros-rate) 10.0)
         (roseus-init t)
         )
   (when roseus-init
     (ros::roseus "jaxon_driving_interface_on_polaris_xp900")
     (ros::rate tmp-ros-rate)
     )
   (send-super :init :motion (instance jaxon-driving-motion-on-polaris-xp900 :init :real nil)
               :real tmp-real :rate tmp-ros-rate :roseus-init nil
               )
   ;; (send self :accel-origin 4)
   ;; (send self :accel-origin -9.69405)
   (send self :step-accel-command 6)
   (send self :release-accel-command 0)
   ;; (send self :step-accel-command 10)
   ;; (send self :release-accel-command 4)
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
