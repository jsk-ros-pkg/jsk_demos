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

  ;; sitting with hip attachment
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

  ;; sitting with left leg out of car
  (:drive-init-pose-half-sitting
   ()
   (send robot :angle-vector
      #f(0.08997 -1.53106 -90.3158 70.1032 -7.83593 1.52475 -3.215627e-06 1.692475e-05 -89.2384 81.6771 9.56135 -1.759062e-05 0.0 0.0 0.0 0.0 0.0 0.0 0.0 -20.0 -90.0 -100.0 -25.0 0.0 -45.0 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
   (send robot :fix-leg-to-coords (make-coords) :lleg)
   (send robot :rleg :move-end-pos (float-vector 0 -350 0) :world)
   (send robot :lleg :move-end-pos (float-vector -300 450 -300) :world :rotation-axis :z :debug-view :no-message :translation-axis :x)
   )
  (:drive-init-pose-touch-fist-to-seat
   (&key (init-pose nil) (tm 20000))
   (when init-pose
     (send self :drive-init-pose-half-sitting)
     )
   ;; (send robot :rarm :shoulder-p :joint-angle 90)
   ;; (send robot :rarm :collar-y :joint-angle 25)
   ;; (send robot :torso :waist-r :joint-angle 8)
   (send robot :angle-vector #f(-15.2035 -27.52 -75.6295 40.9038 2.71831 31.357 29.7154 20.2146 -41.2042 74.219 -31.1754 -19.2137 0.0 0.0 0.0 0.0 0.0 13.419 53.8087 -85.6223 2.91965 -86.7433 22.0021 19.9219 25.9977 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )
   (send robot :angle-vector #f(-15.2035 -27.52 -75.6295 40.9038 2.71831 31.357 29.7154 20.2146 -41.2042 74.219 -31.1754 -19.2137 7.0 0.0 0.0 0.0 0.0 13.7898 60.1655 -15.8257 -1.61586 -95.3469 23.2293 19.935 32.5904 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
   ;; (send robot :rarm :angle-vector #f(13.419 53.8087 -15.8257 2.91965 -86.7433 22.0021 19.9219 25.9977))
   ;; (send robot :rarm :angle-vector #f(13.4283 53.8033 -15.9801 3.11377 -86.7322 21.8079 19.7424 26.06)) ;; 
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )
   )
  (:ride-half-sitting
   ()
   (send robot :move-to (make-coords :pos (float-vector 0 730 -70) :rpy (float-vector 0.0 0.0 0.0)) :world)
   )

  ;; sitting in front of car straightly
  (:drive-init-pose-in-front-of-handle
   ()
   (send robot :angle-vector 
         #f(0.08997 -1.53106 -90.3158 70.1032 -7.83593 1.52475 -3.215627e-06 1.692475e-05 -89.2384 81.6771 9.56135 -1.759062e-05 0.0 0.0 0.0 0.0 0.0 0.0 0.0 -20.0 -90.0 -100.0 -25.0 0.0 -45.0 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0)
         )
   )
  (:ride-in-front-of-handle
   ()
   (send robot :move-to (make-coords :pos (float-vector 30 450 -50) :rpy (float-vector 0.0 0.0 0.0)) :world)
   )
  
  ;; sitting on right of car
  (:drive-init-pose-right-of-car
   ()
   (send robot :angle-vector
         #f(0.08997 -1.53106 -90.3158 70.1032 -7.83593 1.52475 -3.215627e-06 1.692475e-05 -89.2384 81.6771 9.56135 -1.759062e-05 0.0 0.0 0.0 0.0 0.0 0.0 0.0 -20.0 -90.0 -100.0 -25.0 0.0 -45.0 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
   (let* ((lav (copy-seq (send robot :lleg :angle-vector)))
          (rav (copy-seq (send robot :rleg :angle-vector)))
          (laav (copy-seq (send robot :larm :angle-vector)))
          (raav (copy-seq (send robot :rarm :angle-vector))))
     (send robot :rleg :angle-vector lav)
     (send robot :lleg :angle-vector rav)
     ;; (send robot :rarm :angle-vector laav)
     ;; (send robot :larm :angle-vector raav)
     )
   (send robot :larm :shoulder-p :joint-angle 0)
   (send robot :larm :shoulder-y :joint-angle 90)
   (send robot :fix-leg-to-coords (make-coords) :rleg)
   (send robot :rleg :move-end-pos (float-vector 0 -100 0) :local)
   (send robot :lleg :move-end-pos (float-vector 0 400 0) :local)
   )
  (:ride-right-of-car
   ()
   (send robot :move-to (make-coords :pos (float-vector 30 -300 -50) :rpy (float-vector 0.0 0.0 0.0)) :world)
   )

  (:approach-handle-right-of-car
   (&rest args &key (offset #f(150 0 0)) (rot-offset (deg2rad 180)) &allow-other-keys)
   (send-super* :approach-handle :offset offset :rot-offset rot-offset args)
   )
  (:grasp-handle-right-of-car
   (&rest args &key (rotation-axis :z) &allow-other-keys)
   (send-super* :grasp-handle :rotation-axis rotation-axis args)
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

