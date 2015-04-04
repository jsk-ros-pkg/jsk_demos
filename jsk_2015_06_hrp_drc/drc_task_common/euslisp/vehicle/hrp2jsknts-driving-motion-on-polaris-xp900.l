#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(load "package://drc_task_common/euslisp/vehicle/hrp2jsk-driving-motion-on-polaris-xp900.l")

(defclass hrp2jsknts-driving-motion-on-polaris-xp900
  :super hrp2jsk-driving-motion-on-polaris-xp900
  :slots () 
  )

(defmethod hrp2jsknts-driving-motion-on-polaris-xp900
  (:init
   (&key ((:real tmp-real) nil) ((:car tmp-car) nil) (grasp-arm :rarm) (grasp-point :handle-top))
   (hrp2jsknts-init)
   (unless tmp-car
     (setq tmp-car (instance polaris-xp900-with-handle :init :handle-mode :left))
     )
   (send-super :init :robot *hrp2jsknts* :car tmp-car :real tmp-real :grasp-arm grasp-arm :grasp-point grasp-point)
   )
  )

;; (setq *motion* (instance hrp2jsknts-driving-motion-on-polaris-xp900 :init :real nil))

