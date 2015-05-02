#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/staro-driving-motion.l")
(load "package://drc_task_common/euslisp/vehicle/polaris-xp900-with-handle.l")

(defclass staro-driving-motion-on-polaris-xp900
  :super staro-driving-motion
  )

(defmethod staro-driving-motion-on-polaris-xp900
  (:init
   (&key ((:real tmp-real) nil) (grasp-arm :rarm) (grasp-point :handle-top))
   (send-super :init :real tmp-real :car (instance polaris-xp900-with-handle :init) :grasp-arm grasp-arm :grasp-point grasp-point)
   )

  ;; initial methods
  (:ride
   (&key (init-pose t))
   ;; (send robot :move-to (make-coords :pos (float-vector 110 230.0 170) :rpy (float-vector 0.0 0.0 0.0)) :world)
   (send robot :move-to (make-coords :pos (float-vector -100 370.0 20) :rpy (float-vector 0.0 (/ pi 36) 0.0)) :world)
   (when init-pose
     (send self :drive-init-pose)
     )
   )

  (:drive-init-pose-stretch-legs
   (&key (tm 10000))
   (send robot :legs :angle-vector #f(0.0 0.0 -80.0 45.0 -26.0 0.0))
   ;; (send robot :legs :angle-vector #f(0.0 0.0 -86.0 41.0 -1.0 0.0)) ;; chuned angles for accel
   ;; (send robot :legs :angle-vector #f(0.0 0.0 -82.8 41.0 14.0 0.0)) ;; pedaling accel
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )   
   )

  (:drive-init-pose-support-by-leg
   (&key (tm 10000) (limb :lleg))
   ;; (send robot limb :angle-vector #f(0.0 0.0 -86.0 65.0 15.0 0.0))
   ;; (send robot limb :angle-vector #f(-2.592494e-06 3.642189e-05 -84.2409 62.8969 15.344 -3.722199e-05)) ;; move-end-pos #f(0 0 -10)
   (when real
     (send *ri* :start-impedance limb :M-p 100 :D-p 300 :K-p 700)
     )
   (send robot limb :move-end-pos #f(0 0 -10) :world)
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     (send *ri* :set-ref-force #f(0 0 80) 10000 :lleg)
     )
   )

  (:drive-init-pose-exit-car
   (&key (tm 10000))
   ;; (send robot :lleg :angle-vector #f(0.0 0.0 -86.0 65.0 15.0 0.0))
   (send robot :rleg :angle-vector #f(0.0 0.0 -81.0 60.0 15.0 0.0))
   ;; (send robot limb :angle-vector #f(-2.592494e-06 3.642189e-05 -84.2409 62.8969 15.344 -3.722199e-05)) ;; move-end-pos #f(0 0 -10)
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )   
   )
  
  ;; (:approach-accel-pedal
  ;;  (&key (tm 10000) (debug-view nil))
  ;;  ;; (send *staro* :rleg :angle-vector #f(0.0 0.0 -82.8 41.0 14.0 0.0)) ;; chuned parameters for accel
  ;;  (send *staro* :rleg :angle-vector #f(0.0 0.0 -85.0 46.0 0.0 0.0))
  ;;  (when real
  ;;    (send *ri* :angle-vector (send robot :angle-vector) tm)
  ;;    (send *ri* :wait-interpolation)
  ;;    )
  ;;  )
  ;; (:approach-brake-pedal
  ;;  (&key (tm 10000) (debug-view nil))
  ;;  (send *staro* :lleg :angle-vector #f(0.0 0.0 -86.0 65.0 15.0 0.0)) ;; launch to floor
  ;;  (when real
  ;;    (send *ri* :angle-vector (send robot :angle-vector) tm)
  ;;    (send *ri* :wait-interpolation)
  ;;    )
  ;;  )
  (:approach-pedal
   (limb target-pedal &key (tm 2500) (debug-view nil))
   (let (approach-result)
     (case target-pedal
       (:accel-pedal
        (send robot :rleg :angle-vector #f(0.0 0.0 -85.0 46.0 0.0 0.0))
        (setq approach-result t)
        )
       (:brake-pedal
        (send robot :lleg :angle-vector #f(0.0 0.0 -86.0 65.0 15.0 0.0)) ;; launch to floor
        (return-from :approach-pedal nil)
        )
       (t
        (warn "Invalid pedal name~%")
        (return-from :approach-pedal nil)
        )
       )
     (when (and real approach-result)
       (send *ri* :angle-vector (send robot :angle-vector) tm)
       (send *ri* :wait-interpolation)
       )
     )
   )
  
  )

;; (setq *motion* (instance staro-driving-motion-on-polaris-xp900 :init :real nil))
