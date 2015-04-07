#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
(load "package://drc_task_common/euslisp/vehicle/robot-driving-motion.l")
(load "package://drc_task_common/euslisp/vehicle/polaris-xp900-with-handle.l")

(defclass jaxon-driving-motion-on-polaris-xp900
  :super robot-driving-motion
  )

(defmethod jaxon-driving-motion-on-polaris-xp900
  (:init
   (&key ((:car tmp-car) nil) ((:real tmp-real) nil) (grasp-arm :rarm) (grasp-point :handle-top))
   (jaxon-init)
   (unless tmp-car
     (setq tmp-car (instance polaris-xp900-with-handle :init :handle-mode :left))
     )
   (send-super :init :robot *jaxon* :car tmp-car :real tmp-real :grasp-arm grasp-arm :grasp-point grasp-point)
   )
  (:ride
   (&key (init-pose t))
   ;; for jaxon
   ;; (send robot :move-to (make-coords :pos (float-vector 550 370.0 20) :rpy (float-vector 0.0 (- (/ pi 6)) 0.0)) :world)
   ;; (send robot :move-to (make-coords :pos (float-vector 680 370.0 150) :rpy (float-vector 0.0 (- (/ pi 5)) 0.0)) :world)
   ;; (send robot :move-to (make-coords :pos (float-vector 650 370.0 120) :rpy (float-vector 0.0 (- (/ pi 5)) 0.0)) :world) ;; in front of accel
   ;; (send robot :move-to (make-coords :pos (float-vector 650 450.0 120) :rpy (float-vector 0.0 (- (/ pi 5)) 0.0)) :world) ;; in front of handle cog is on the center of trapezoid
   ;; (send robot :move-to (make-coords :pos (float-vector 550 450.0 150) :rpy (float-vector 0.0 (- (/ pi 6)) 0.0)) :world) ;; cog is on the front edge of trapezoid
   ;; (send robot :move-to (make-coords :pos (float-vector 500 450.0 100) :rpy (float-vector 0.0 (- (deg2rad 24.0)) 0.0)) :world) ;; cog is on the front edge of trapezoid
   (send robot :move-to (make-coords :pos (float-vector 500 450 30) :rpy (float-vector 0.0 (- (deg2rad 24.0)) 0.0)) :world) ;; fix z coords for BODY 24deg version
   (when init-pose
     (send self :drive-init-pose)
     )
   )
  (:drive-init-pose
   (&key (tm 20000))
   (send robot :reset-manip-pose)
   (send robot :legs :crotch-p :joint-angle -85)
   (send robot :legs :knee-p :joint-angle 80)
   (send robot :legs :ankle-p :joint-angle 0)
   ;; (send robot :rarm :shoulder-p :joint-angle -100)
   (send robot :rarm :shoulder-p :joint-angle 0)

   ;; (send robot :torso :waist-p :joint-angle 30)
   ;; (send robot :legs :crotch-p :joint-angle -65)
   ;; (send robot :legs :knee-p :joint-angle 100)
   (send robot :torso :waist-p :joint-angle 20)
   (send robot :legs :crotch-p :joint-angle -70)
   (send robot :legs :knee-p :joint-angle 90)
   (send robot :legs :ankle-p :joint-angle 2)
   
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )
   )
  (:open-hand
   (limb)
   (when real
     ;; (send *ri* :stop-grasp limb)
     ;; (send *ri* :hand-open limb)
     )
   )
  (:close-hand
   (limb)
   (when real
     ;; (send *ri* :start-grasp limb)
     ;; (send *ri* :hand-close limb)
     )
   )
  (:set-impedance-for-handle
   (limb)
   (send *ri* :start-impedance limb :moment-gain #f(0 0 0) :D-p 400 :K-p 800)
   )
  ;; (:approach-accel-pedal
  ;;  (&key (tm 10000) (debug-view nil))
  ;;  ;; (send robot :rleg :angle-vector #f(0.271875 0.261 -86.08 62.4348 2.384 -0.234 -0.384)) ;; adjusted joint-angles in real hrp2jsknt
  ;;  (send robot :rleg :angle-vector #f(0.269775 0.256892 -84.3383 59.7212 -4.02795 -0.229299 -0.384)) ;; adjusted joint-angles in real hrp2jsknt   
  ;;  (when real
  ;;    (send *ri* :angle-vector (send robot :angle-vector) tm)
  ;;    (send *ri* :wait-interpolation)
  ;;    )
  ;;  )
  ;; (:approach-brake-pedal
  ;;  (&key (tm 10000) (debug-view nil))
  ;;  (send robot :lleg :angle-vector #f(0.0375 -0.5235 -83.014 80.541 9.842 0.6048 -0.564)) ;; adjusted joint-angles in real hrp2jsknt
  ;;  (when real
  ;;    (send *ri* :angle-vector (send robot :angle-vector) tm)
  ;;    (send *ri* :wait-interpolation)
  ;;    )
  ;;  )
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

;; (setq *motion* (instance jaxon-driving-motion-on-polaris-xp900 :init :real nil))

