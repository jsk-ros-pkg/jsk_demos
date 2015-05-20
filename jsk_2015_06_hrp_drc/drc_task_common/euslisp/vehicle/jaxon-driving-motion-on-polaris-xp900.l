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
   (&key (tm 10000))
   ;; (send robot :angle-vector #f(-13.1189 -24.4656 -83.3638 44.9385 7.33334 27.7739 15.5265 -14.998 -95.4989 97.065 0.433944 15.5246 8.0 0.0 0.0 0.0 0.0 17.0643 79.1999 -96.2707 -5.3561 -85.9563 27.0803 22.0044 22.1802 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
   ;; (send robot :angle-vector #f(-13.1189 -24.4656 -83.3638 44.9385 7.33334 27.7739 15.5265 -14.998 -95.4989 97.065 0.433944 15.5246 8.0 0.0 0.0 0.0 0.0 -13.7511 33.5688 -108.234 8.54591 -90.0 0.0 45.0 3.38633 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0)) ;; fixed pose in driving-simulator
   ;; (send robot :angle-vector #f(-13.2762 -24.7025 -92.0869 71.704 -10.7745 28.0506 36.0874 -21.6131 -90.5514 112.587 -19.1011 14.1156 8.0 0.0 0.0 0.0 0.0 -13.7511 33.5688 -108.234 8.54591 -90.0 0.0 45.0 3.38633 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0)) ;; move legs -100 in x axis from above
   ;; (send robot :angle-vector #f(-10.8755 -20.9737 -94.5136 76.5544 -12.2659 23.7151 35.779 -15.8163 -94.2405 111.319 -14.2426 8.31144 8.0 0.0 0.0 0.0 0.0 -13.7511 33.5688 -108.234 8.54591 -90.0 0.0 45.0 3.38633 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0)) ;; move torso to -50
   ;; (send robot :angle-vector #f(-8.43548 -16.9475 -96.4631 80.3675 -13.336 19.0749 35.4682 -9.61909 -96.8849 109.094 -9.44266 2.10682 8.0 0.0 0.0 0.0 0.0 -13.7511 33.5688 -108.234 8.54591 -90.0 0.0 45.0 3.38633 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0)) ;; move torso to -100
   ;; (send robot :angle-vector #f(-13.7393 -25.3936 -95.4094 71.4519 -7.39605 28.8591 30.4376 -19.3225 -88.6644 109.485 -20.8208 19.3216 5.68288 0.0 -0.472369 0.0 0.0 -0.900602 25.4208 -73.4803 49.5891 -47.773 -0.036939 13.1557 -20.6728 -31.1749 -18.8595 28.6218 -40.0612 -119.661 11.6674 -22.9237 11.7398)) ;; noda-sitting-pose
   ;; (send robot :angle-vector #f(-13.7323 -25.8376 -95.7889 82.4073 -16.562 29.2379 30.4377 -18.1196 -87.6194 110.528 -22.9086 18.1188 5.68288 0.0 -0.472369 -32.0 34.74 0.13164 24.9727 -71.0757 49.82 -44.5932 0.487797 12.7499 -21.7149 -26.8661 -14.9803 34.5785 -38.4324 -123.493 13.0696 -25.4864 6.78645)) ;; noda-sitting-pose-2
   (send robot :angle-vector #f(-13.7323 -25.8376 -95.7889 82.4073 -16.562 29.2379 30.4377 -18.1196 -87.6194 110.528 -22.9086 18.1188 5.68288 0.0 -0.472369 0.0 0.0 0.13164 24.9727 -71.0757 49.82 -44.5932 0.487797 12.7499 -21.7149 -26.8661 -14.9803 34.5785 -38.4324 -123.493 13.0696 -25.4864 6.78645)) ;; noda-sitting-pose-2 with neck 0
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
   ;; (send robot :move-to (make-coords :pos (float-vector 20 720 -90) :rpy (float-vector 0.0 0.0 0.0)) :world) ;; adjust in real vehicle
   ;; (send robot :move-to (make-coords :pos (float-vector 20 670 -90) :rpy (float-vector 0.0 0.0 0.0)) :world) ;; move torso to -50
   ;; (send robot :move-to (make-coords :pos (float-vector 20 620 -90) :rpy (float-vector 0.0 0.0 0.0)) :world) ;; move torso to -100
   (send robot :move-to (make-coords :pos (float-vector 20 720 -65) :rpy (float-vector 0.0 0.0 0.0)) :world) ;; noda-sitting-pose and fixed in real polaris
   )

  (:turn-handle
   (deg &rest args &key (stop 100) &allow-other-keys)
   (send-super* :turn-handle deg :stop stop args)
   )
  (:open-hand
   (&optional (limb (send self :grasp-arm)))
   (when real
     (send *ri* :stop-grasp limb)
     ;; (send *ri* :hand-open limb)
     )
   )
  (:close-hand
   (&optional (limb (send self :grasp-arm)))
   (when real
     (send *ri* :start-grasp limb)
     ;; (send *ri* :hand-close limb)
     )
   )
  (:set-impedance-for-handle
   (limb)
   ;; (send *ri* :start-impedance limb :M-p 200 :D-p 600 :K-p 1400)
   ;; (send *ri* :start-impedance limb :M-p 200 :D-p 1200 :K-p 2000)
   (send *ri* :start-impedance limb :M-p 0 :D-p 1200 :K-p 2000 :M-r 0.0) ;; M = 0 temporarily for impedance follow-up
   )
  (:set-impedance-for-support
   (limb)
   (cond
    ((member limb (list :larm :rarm :arms))
     ;; (send *ri* :start-impedance limb :M-p 150 :D-p 2000 :K-p 4000 :moment-gain #(1 1 1) :M-r 25 :D-r 150 :K-r 200)
     (send *ri* :start-impedance limb :M-p 0 :D-p 2000 :K-p 4000 :moment-gain #(1 1 1) :M-r 0 :D-r 150 :K-r 200) ;; M = 0 temporarily for impedance follow-up
     )
    ((member limb (list :lleg :rleg :legs))
     ;; (send *ri* :start-impedance limb :M-p 200 :D-p 4000 :K-p 5000 :moment-gain #(1 1 1) :M-r 50 :D-r 500 :K-r 600)
     (send *ri* :start-impedance limb :M-p 0 :D-p 4000 :K-p 5000 :moment-gain #(1 1 1) :M-r 0 :D-r 500 :K-r 600) ;; M = 0 temporarily for impedance follow-up
     )
    (t ;; default
     ;; (send *ri* :start-impedance limb :M-p 200 :D-p 4000 :K-p 6000 :moment-gain #(1 1 1) :M-r 50 :D-r 250 :K-r 400)
     (send *ri* :start-impedance limb :M-p 0 :D-p 4000 :K-p 6000 :moment-gain #(1 1 1) :M-r 0 :D-r 250 :K-r 400) ;; M = 0 temporarily for impedance follow-up
     )
    )
   )

  (:approach-pedal
   (limb target-pedal &key (tm 2000) (debug-view nil))
   (let (approach-result)
     (case target-pedal
       (:accel-pedal
        ;; (send robot :rleg :angle-vector #f(-13.1189 -24.4656 -83.3638 44.9385 7.33334 27.7739)) ;; rleg angle-vector in left-sitting
        ;; (send robot :rleg :angle-vector #f(-13.2762 -24.7025 -92.0869 71.704 -10.7745 28.0506)) ;; move legs -100 in x axis from above
        ;; (send robot :rleg :angle-vector #f(-10.8755 -20.9737 -94.5136 76.5544 -12.2659 23.7151)) ;; move torso -50
        ;; (send robot :rleg :angle-vector #f(-8.43548 -16.9475 -96.4631 80.3675 -13.336 19.0749)) ;; move torso -100
        ;; (send robot :rleg :angle-vector #f(-8.75369 -17.4852 -99.9228 80.091 -9.69405 19.6926)) ;; fixed in real polaris
        ;; (send robot :rleg #f(30.4376 -19.3225 -88.6644 109.485 -20.8208 19.3216)) ;; noda-sitting-pose
        (send robot :rleg :angle-vector #f(-12.8499 -24.172 -89.5563 69.4056 -13.128 27.3947)) ;; noda-sitting-pose fixed in real polaris
        (setq approach-result t)
        )
       (:brake-pedal
        (warn "Disable approach-brake-pedal for JAXON~%")
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
     approach-result
     )
   )

  (:approach-fist
   (&rest args &key (rotation-axis :x) (reach-until-touch-thre 80) (max-dist 100) &allow-other-keys)
   (send-super* :approach-fist :rotation-axis rotation-axis :reach-until-touch-thre reach-until-touch-thre :max-dist max-dist args)
   )
  (:approach-floor
   (&rest args &key (start-impedance nil) (offset #f(0 0 10)) (reach-until-touch-thre 150) (max-dist 100) &allow-other-keys)
   (send-super* :approach-floor :start-impedance start-impedance :offset offset :reach-until-touch-thre reach-until-touch-thre :max-dist max-dist args)
   )

  ;; egress
  (:egress-pose
   ()
   (send robot :angle-vector #f(-13.1189 -24.4656 -83.3638 44.9385 7.33334 27.7739 15.5265 -14.998 -95.4989 97.065 0.433944 15.5246 11.3761 0.0 0.0 0.0 0.0 13.4236 66.4673 -16.1854 -4.76865 -105.284 22.9836 20.8474 38.8758 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
   )
  (:go-to-egress
   (&rest args &key (time 10000))
   (let (fist-arm)
     (if (equal (send self :grasp-arm) :larm)
         (setq fist-arm :rarm)
       (setq fist-arm :larm)
       )
     (send self :turn-handle 0 :relative nil)
     (send self :approach-fist :limb fist-arm :offset #f(0 0 100) :offset-wrt :world :reach-until-touch nil :time time)
     (send robot fist-arm :angle-vector #f(13.4236 66.4673 -16.1854 -4.76865 -105.284 22.9836 20.8474 38.8758)) ;; send rarm angle-vector of egress-pose
     (when real
       (send *ri* :angle-vector (send robot :angle-vector) time)
       (send *ri* :wait-interpolation)
       )
     )
   )
  )


;; obsoluted drive-init-pose
(defmethod jaxon-driving-motion-on-polaris-xp900
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
  (:drive-init-pose-grasp-misumi-frame-attachment
   (&key (tm 20000))
   ;; (send robot :angle-vector #f(-16.788 -29.688 -85.152 51.324 1.10001 35.526 34.32 28.08 -43.86 91.176 -45.98 -27.047 0.015 0.0 0.0 -0.00625 -0.00625 -8.80625 -22.9125 -120.362 104.919 -35.7687 -121.669 84.2437 23.9627 0.0 54.9937 19.9875 14.9937 -99.9875 24.9937 -0.00625 -44.9938))
   (send robot :angle-vector #f(-16.788 -29.688 -85.152 51.324 1.10001 35.526 34.32 28.08 -43.86 91.176 -45.98 -27.047 8.0 0.0 0.0 -0.00625 -0.00625 -6.13953 -23.8666 -141.172 98.7428 -51.9587 -113.096 79.8593 21.5153 0.0 54.9937 19.9875 14.9937 -99.9875 24.9937 -0.00625 -44.9938)) ;; bend waist-r
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

  ;; mid pose of sitting and egressing
  (:drive-init-pose-pre-left-sitting
   ()
   (send robot :angle-vector #f(-8.78949 -17.5481 -85.7058 61.2531 -5.08475 19.7642 30.8982 -27.0106 -81.7285 82.775 0.897663 28.0243 0.0 0.0 0.0 0.0 0.0 17.0643 79.1999 -96.2707 -5.3561 -85.9563 27.0803 22.0044 22.1802 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
   )
  (:drive-init-pose-left-sitting
   ()
   ;; (send robot :angle-vector #f(-8.78949 -17.5481 -85.7058 61.2531 -5.08475 19.7642 30.8982 -27.0106 -81.7285 82.775 0.897663 28.0243 0.0 0.0 0.0 0.0 0.0 20.8944 56.1786 -15.8251 -5.3561 -85.9563 27.0803 22.0044 22.1802 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
   ;; (send robot :angle-vector #f(-13.1189 -24.4656 -83.3638 44.9385 7.33334 27.7739 15.5265 -14.998 -95.4989 97.065 0.433944 15.5246 11.3761 0.0 0.0 0.0 0.0 13.4594 70.6352 -15.8262 -6.09977 -111.857 21.973 21.9569 41.3552 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0)) ;; adjsut in real vehicle
   (send robot :angle-vector #f(-13.1189 -24.4656 -83.3638 44.9385 7.33334 27.7739 15.5265 -14.998 -95.4989 97.065 0.433944 15.5246 11.3761 0.0 0.0 0.0 0.0 13.4236 66.4673 -16.1854 -4.76865 -105.284 22.9836 20.8474 38.8758 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
   )
  (:ride-left-sitting
   ()
   ;; (send robot :move-to (make-coords :pos (float-vector 20 580 -50) :rpy (float-vector 0.0 0.0 0.0)) :world)
   ;; (send robot :move-to (make-coords :pos (float-vector 20 720 -120) :rpy (float-vector 0.0 0.0 0.0)) :world) ;; adjust in real vehicle
   (send robot :move-to (make-coords :pos (float-vector 20 720 -90) :rpy (float-vector 0.0 0.0 0.0)) :world) ;; adjust in real vehicle
   )
  (:drive-init-pose-left-sitting-all
   (&key (time 10000))
   (send self :drive-init-pose-pre-left-sitting)
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) time)
     (send *ri* :wait-interpolation)
     )
   (send self :drive-init-pose-left-sitting)
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) time)
     (send *ri* :wait-interpolation)
     )
   )
  )

;; obsoluted egress function
(defmethod jaxon-driving-motion-on-polaris-xp900
  (:prepare-get-off
   ()
   (send *jaxon* :lleg :move-end-pos #f(0 0 40) :world)
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) time)
     (send *ri* :wait-interpolation)
     )
   (send *jaxon* :lleg :move-end-pos #f(0 200 0) :world)
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) time)
     (send *ri* :wait-interpolation)
     )
   )
  )

;; (setq *motion* (instance jaxon-driving-motion-on-polaris-xp900 :init :real nil))

;; :rleg :angle-vector #f(-8.91258 -17.7523 -100.784 83.8894 -12.6788 19.9996)
;; x +20 z +30 #f(-8.75369 -17.4852 -99.9228 80.091 -9.69405 19.6926)

;; succeeded lasvegas pose 20150515
;; fullbody (send robot :angle-vector #f(-8.43548 -16.9475 -96.4631 80.3675 -13.336 19.0749 35.4682 -9.61909 -96.8849 109.094 -9.44266 2.10682 8.0 0.0 0.0 0.0 0.0 -13.7511 33.5688 -108.234 8.54591 -90.0 0.0 45.0 3.38633 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0)) ;; move torso to -100
;; accel (send robot :rleg :angle-vector #f(-8.75369 -17.4852 -99.9228 80.091 -9.69405 19.6926)) ;; fixed in real polaris
;; ride (send robot :move-to (make-coords :pos (float-vector 20 620 -90) :rpy (float-vector 0.0 0.0 0.0)) :world) ;; move torso to -100
