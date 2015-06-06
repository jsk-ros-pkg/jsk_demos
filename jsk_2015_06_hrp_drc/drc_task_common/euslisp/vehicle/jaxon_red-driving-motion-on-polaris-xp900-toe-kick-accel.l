#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(load "package://drc_task_common/euslisp/vehicle/jaxon-driving-motion-on-polaris-xp900-toe-kick-accel.l")

(defclass jaxon_red-driving-motion-on-polaris-xp900
  :super jaxon-driving-motion-on-polaris-xp900
  )

(defmethod jaxon_red-driving-motion-on-polaris-xp900
  (:init
   (&key ((:car tmp-car) nil) ((:real tmp-real) nil) (grasp-arm :larm) (grasp-point :handle-crank))
   (jaxon_red-init)
   (unless tmp-car
     (setq tmp-car (instance polaris-xp900-with-handle :init))
     )
   (send-super :init :robot *jaxon_red* :car tmp-car :real tmp-real :grasp-arm grasp-arm :grasp-point grasp-point)
   )    
  )

;; (setq *motion* (instance jaxon-driving-motion-on-polaris-xp900 :init :real nil))
