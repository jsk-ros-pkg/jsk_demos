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
         )
   (when roseus-init
     (ros::roseus "staro_driving_controller_on_polaris_xp900")
     (ros::rate tmp-ros-rate)
     )
   (send-super :init :motion (instance staro-driving-motion-on-polaris-xp900 :init :real nil
                                       :grasp-arm :rarm :grasp-point :handle-crank)
               :real tmp-real :rate tmp-ros-rate :roseus-init nil
               )
   ;; set default accel angle
   ;; (send self :accel-origin 0)
   (send self :step-accel-command 20)
   (send self :release-accel-command 0)
   )
  (:brake-cmd
   (cmd)
   (warn "Real Vehicle Mode: Brake Disabled~%")
   (send self :accel-cmd 0.0) ;; release accel forcely
   )
  )

(defun staro-vehicle-main
  (&key (real t))
  (robot-vehicle-main :robot-name "staro" :postfix "on-polaris-xp900" :real real)
  )

;; (setq *controller* (instance staro-driving-controller-on-polaris-xp900 :init :real nil))
