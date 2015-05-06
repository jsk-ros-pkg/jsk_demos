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
  (:open-hand
   (&optional (limb (send self :grasp-arm)))
   (when real
     (send robot :hand limb :hook-pose)
     (send *ri* :hand-angle-vector (send robot :hand-angle-vector) 1000)
     (send *ri* :wait-interpolation)
     )
   )
  (:close-hand
   (&optional (limb (send self :grasp-arm)))
   (when real
     ;; (send robot :hand limb :close-pose)
     ;; (send robot :hand limb :angle-vector #f(65.0 0.0 0.0 0.0 10.0 90.0)) ;; do not use index finger
     (send robot :hand limb :grasp-pose)
     (send *ri* :hand-angle-vector (send robot :hand-angle-vector) 1000)
     (send *ri* :wait-interpolation)
     )
   )
  )

;; (setq *motion* (instance hrp2jsknts-driving-motion-on-polaris-xp900 :init :real nil))

