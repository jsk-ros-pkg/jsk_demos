#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
(load "package://drc_task_common/euslisp/vehicle/robot-driving-motion.l")
(load "package://drc_task_common/euslisp/vehicle/polaris-xp900-with-handle.l")

(defclass hrp2jsk-driving-motion-on-polaris-xp900
  :super robot-driving-motion
  )

(defmethod hrp2jsk-driving-motion-on-polaris-xp900
  (:init
   (&key ((:robot tmp-robot) nil) ((:car tmp-car) nil) ((:real tmp-real) nil) (grasp-arm :rarm) (grasp-point :handle-top))
   (unless tmp-robot
     (hrp2jsk-init)
     (setq tmp-robot *hrp2jsk*)
     )
   (unless tmp-car
     (setq tmp-car (instance polaris-xp900-with-handle :init :handle-mode :left))
     )
   (send-super :init :robot tmp-robot :car tmp-car :real tmp-real :grasp-arm grasp-arm :grasp-point grasp-point)
   )
  (:ride
   (&key (init-pose t))
   ;; (send robot :move-to (make-coords :pos (float-vector 0 370.0 200) :rpy (float-vector 0.0 0.0 0.0)) :world)
   ;; (send robot :move-to (make-coords :pos (float-vector 180 370.0 80) :rpy (float-vector 0.0 (- (/ pi 16)) 0.0)) :world)
   (send robot :move-to (make-coords :pos (float-vector -80 370.0 80) :rpy (float-vector 0.0 0.0 0.0)) :world)
   (when init-pose
     (send self :drive-init-pose)
     )
   )
  (:drive-init-pose ;; todo: merge approach-brake pose
   (&key (tm 20000))
   ;; (send robot :reset-manip-pose)
   ;; (send robot :legs :angle-vector #f(0.0 0.0 -90.0 64.0 6.5 0.0 0.0))
   (send robot :reset-manip-pose)
   (send robot :legs :crotch-p :joint-angle -85)
   (send robot :legs :knee-p :joint-angle 80)
   (send robot :legs :ankle-p :joint-angle 0)
   (send robot :rarm :shoulder-p :joint-angle -100)
   ;; (send robot :torso :waist-p :joint-angle 10)
   (send robot :head :neck-p :joint-angle -5)
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )
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
     (send robot :hand limb :close-pose)
     (send *ri* :hand-angle-vector (send robot :hand-angle-vector) 1000)
     (send *ri* :wait-interpolation)
     )
   )
  (:approach-handle
   (&rest args &key (offset #f(-50 30 30)) &allow-other-keys)
   (send-super* :approach-handle :offset offset args)
   )
  (:set-impedance-for-handle
   (limb)
   (send *ri* :start-impedance limb :moment-gain #f(0 0 0) :D-p 400 :K-p 800)
   )
  (:approach-accel-pedal
   (&key (tm 10000) (debug-view nil))
   ;; (send robot :rleg :angle-vector #f(0.271875 0.261 -86.08 62.4348 2.384 -0.234 -0.384)) ;; adjusted joint-angles in real hrp2jsknt
   ;; (send robot :rleg :angle-vector #f(0.269905 0.257049 -84.7676 56.5701 -0.447684 -0.229449 -0.384)) ;; adjusted joint-angles in real hrp2jsknt for new seat (testbed version)
   ;; (send robot :rleg :angle-vector #f(0.274637 0.265758 -88.9665 55.0 -2.68319 -0.239387 -0.384)) ;; adjusted joint-angles in real hrp2jsknt for new seat (testbed version)
   (send robot :rleg :angle-vector #f(0.277593 0.269639 -91.1699 48.9728 5.54723 -0.244199 -0.384))
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )
   )
  (:approach-brake-pedal
   (&key (tm 10000) (debug-view nil))
   ;; (send robot :lleg :angle-vector #f(0.0375 -0.5235 -83.014 80.541 9.842 0.6048 -0.564)) ;; adjusted joint-angles in real hrp2jsknt
   (send robot :lleg :angle-vector #f(0.398437 -1.4895 -97.47 76.7502 20.0 1.5876 -1.548))
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     (send self :set-impedance-for-support :lleg)
     )
   )
  (:step-accel-pedal
   (move-mm &key (time 1000) (relative t) (use-controller t))
   (send robot :rleg :move-end-pos (float-vector 0 0 move-mm) :local)
   (when real
     (if use-controller
         (send *ri* :angle-vector (send robot :angle-vector) time :rleg-controller)
       (send *ri* :angle-vector (send robot :angle-vector) time)
       )
     (send *ri* :wait-interpolation)
     )
   )
  )

;; (setq *motion* (instance hrp2jsk-driving-motion-on-polaris-xp900 :init :real nil))

