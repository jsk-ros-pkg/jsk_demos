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
  
  (:drive-init-pose
   (&key (tm 20000))
   (send self :drive-init-pose-half-sitting)
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )
   )
  (:ride
   (&key (init-pose t))
   (when init-pose
     (send self :drive-init-pose)
     )
   (send self :ride-half-sitting)
   )

  (:drive-init-pose-hip-attachment
   ()
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
   )
  (:ride-hip-attachment
   ()
   ;; for jaxon
   (send robot :move-to (make-coords :pos (float-vector 500 450 30) :rpy (float-vector 0.0 (- (deg2rad 24.0)) 0.0)) :world) ;; fix z coords for BODY 24deg version
   )
  
  (:drive-init-pose-half-sitting
   ()
   (send robot :angle-vector
      #f(0.08997 -1.53106 -90.3158 70.1032 -7.83593 1.52475 -3.215627e-06 1.692475e-05 -89.2384 81.6771 9.56135 -1.759062e-05 0.0 0.0 0.0 0.0 0.0 0.0 0.0 -20.0 -90.0 -100.0 -25.0 0.0 -45.0 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
   (send robot :fix-leg-to-coords (make-coords) :lleg)
   (send robot :rleg :move-end-pos (float-vector 0 -350 0) :world)
   (send robot :lleg :move-end-pos (float-vector -300 450 -300) :world :rotation-axis :z :debug-view :no-message :translation-axis :x)
   )
  (:ride-half-sitting
   ()
   (send robot :move-to (make-coords :pos (float-vector 0 730 -70) :rpy (float-vector 0.0 0.0 0.0)) :world)
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
  (:approach-handle
   (&key (limb (send self :get :grasp-arm)) (target-handle (send self :get :grasp-point)) (sync-controller nil)
         (tm 15000) (offset #f(-50 0 0)) (debug-view nil) (rotation-axis t) (start-impedance nil) (stop-impedance nil)
         (null-space #f(-0.06 -0.3 0.0 -0.18 0.16 0.0 0.0 0.0))
         )
   (send-super :approach-handle
               :limb limb :target-handle target-handle :sync-controller sync-controller
               :tm tm :offset offset :debug-view debug-view :rotation-axis rotation-axis
               :start-impedance start-impedance :stop-impedance stop-impedance :null-space null-space
               )
   )
  (:set-impedance-for-handle
   (limb)
   (send *ri* :start-impedance limb :M-p 200 :D-p 600 :K-p 1400)
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
  ;; (:step-accel-pedal
  ;;  (move-mm &key (time 1000) (relative t) (use-controller t))
  ;;  (send robot :rleg :move-end-pos (float-vector 0 0 move-mm) :local)
  ;;  (when real
  ;;    (if use-controller
  ;;        (send *ri* :angle-vector (send robot :angle-vector) time :rleg-controller)
  ;;      (send *ri* :angle-vector (send robot :angle-vector) time)
  ;;      )
  ;;    (send *ri* :wait-interpolation)
  ;;    )
  ;;  )
  )

;; (setq *motion* (instance jaxon-driving-motion-on-polaris-xp900 :init :real nil))

