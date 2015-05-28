#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/jaxon-driving-controller-on-polaris-xp900.l")
(load "package://drc_task_common/euslisp/vehicle/jaxon_red-driving-motion-on-polaris-xp900.l")

(defclass jaxon_red-driving-controller-on-polaris-xp900
  :super jaxon-driving-controller-on-polaris-xp900
  )

(defmethod jaxon_red-driving-controller-on-polaris-xp900
  (:init
   (&key ((:real tmp-real) nil) ((:rate tmp-ros-rate) 10.0) (roseus-init t)
         )
   (when roseus-init
     (ros::roseus "jaxon_red_driving_interface_on_polaris_xp900")
     (ros::rate tmp-ros-rate)
     )
   (send-super :init :motion (instance jaxon_red-driving-motion-on-polaris-xp900 :init :real nil)
               :real tmp-real :rate tmp-ros-rate :roseus-init nil
               )
   )
  )

(defun jaxon_red-vehicle-main
  (&key (real t))
  (robot-vehicle-main :robot-name "jaxon_red" :postfix "on-polaris-xp900" :real real)
  )

;; (setq *controller* (instance jaxon_red-driving-controller-on-polaris-xp900 :init :real nil))
