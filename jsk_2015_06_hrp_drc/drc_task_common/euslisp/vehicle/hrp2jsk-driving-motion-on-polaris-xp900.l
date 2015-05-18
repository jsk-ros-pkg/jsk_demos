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
   ;; (send robot :move-to (make-coords :pos (float-vector -80 370.0 80) :rpy (float-vector 0.0 0.0 0.0)) :world)
   ;; (send robot :move-to (make-coords :pos (float-vector -80 470.0 80) :rpy (float-vector 0.0 0.0 0.0)) :world) ;; condier egress in driving-simulator
   ;; (send robot :move-to (make-coords :pos (float-vector -80 470.0 180) :rpy (float-vector 0.0 (deg2rad 5.0) 0.0)) :world) ;; adjusted in real polaris in lasvegas
   ;; (send robot :move-to (make-coords :pos (float-vector -80 420.0 180) :rpy (float-vector 0.0 (deg2rad 5.0) 0.0)) :world)
   (send robot :move-to (make-coords :pos (float-vector -140 420.0 180) :rpy (float-vector 0.0 (deg2rad 5.0) 0.0)) :world)
   (when init-pose
     (send self :drive-init-pose)
     )
   )
  (:drive-init-pose ;; todo: merge approach-brake pose
   (&key (tm 20000))
   ;; ;; (send robot :reset-manip-pose)
   ;; ;; (send robot :legs :angle-vector #f(0.0 0.0 -90.0 64.0 6.5 0.0 0.0))
   ;; (send robot :reset-manip-pose)
   ;; (send robot :legs :crotch-p :joint-angle -85)
   ;; (send robot :legs :knee-p :joint-angle 80)
   ;; (send robot :legs :ankle-p :joint-angle 0)
   ;; ;; (send robot :rarm :shoulder-p :joint-angle -100)
   ;; ;; (send robot :torso :waist-p :joint-angle 10)
   ;; (send robot :head :neck-p :joint-angle -5)
   ;; ;; (send robot :lleg :angle-vector #f(0.398437 -1.4895 -97.47 76.7502 20.0 1.5876 -1.548)) ;; use pedaling block for hrp2
   ;; ;; (send robot :lleg :angle-vector #f(0.059286 7.53038 -101.538 72.9173 30.1805 -7.533 -1.548)) ;; fix for egress
   ;; (send robot :lleg :angle-vector #f(5.00812 -1.07931 -97.6734 68.4796 28.4724 1.1772 -1.548)) ;; fix for egress using crotch
   ;; (send robot :angle-vector #f(0.0 0.0 -85.0 80.0 0.0 0.0 0.0 5.00812 -1.07931 -97.6734 68.4796 28.4724 1.1772 -1.548 0.0 0.0 0.0 -5.0 50.0 -30.0 -10.0 -120.0 -25.0 -5.0 -20.0 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0)) ;; old drive-init-pose adjusted in drive-simulation-environment

   (send robot :reset-manip-pose) ;; adjusted in real polaris in lasvegas

   ;; (send robot :lleg :angle-vector #f(4.8375 -1.095 -88.646 72.6426 10.4 1.0224 0.216))
   ;; (send robot :rleg :angle-vector #f(-9.04768 1.17199 -100.84 58.9158 10.7088 -1.14559 -0.108))
   ;; (send robot :rarm :angle-vector #f(35.9754 -42.1297 8.69405 -74.6325 -43.4673 3.3033 35.8167 0.0))

   (send robot :lleg :angle-vector #f(5.66017 7.49564 -86.8436 69.5621 12.0391 -7.95977 0.216))
   (send robot :rleg :angle-vector #f(-5.32243 7.27429 -102.357 61.9889 8.87751 -8.28873 -0.108))
   (send robot :rarm :angle-vector #f(35.9754 -42.1297 8.69405 -74.6325 -43.4673 3.3033 35.8167 0.0))
   
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )
   )
  (:open-hand
   (&optional (limb (send self :grasp-arm)))
   (when real
     (send robot limb :thumb-r :joint-angle -90) ;; hrp2jsk do not have hrp3hand
     (send *ri* :angle-vector (send robot :angle-vector) 1000)
     (send *ri* :wait-interpolation)
     )
   )
  (:close-hand
   (&optional (limb (send self :grasp-arm)))
   (when real
     (send robot limb :thumb-r :joint-angle 0) ;; hrp2jsk do not have hrp3hand
     (send *ri* :angle-vector (send robot :angle-vector) 1000)
     (send *ri* :wait-interpolation)
     )
   )
  (:approach-handle
   (&rest args &key (offset #f(-50 30 30)) &allow-other-keys)
   (send-super* :approach-handle :offset offset args)
   )
  (:approach-floor
   (&rest args &key (target-handle :floor-footrest) &allow-other-keys)
   (send-super* :approach-floor :target-handle target-handle args)
   )
  (:set-impedance-for-handle
   (limb)
   ;; (send *ri* :start-impedance limb :moment-gain #f(0 0 0) :D-p 400 :K-p 800)
   (send *ri* :start-impedance limb :moment-gain #f(0 0 0) :M-p 0.0 :D-p 400 :K-p 800) ;; M = 0 temporarily for impedance follow-up
   )
  (:set-impedance-for-support
   (limb)
   (cond
    ((member limb (list :larm :rarm :arms))
     ;; (send *ri* :start-impedance limb :M-p 150 :D-p 2000 :K-p 4000 :moment-gain #(1 1 1) :M-r 25 :D-r 150 :K-r 200)
     (send *ri* :start-impedance limb :M-p 0 :D-p 2000 :K-p 4000 :moment-gain #(1 1 1) :M-r 0 :D-r 150 :K-r 200) ;; M = 0 temporarily for impedance follow-up
     )
    ((member limb (list :lleg :rleg :legs))
     ;; (send *ri* :start-impedance limb :M-p 200 :D-p 3000 :K-p 5000 :moment-gain #(1 1 1) :M-r 50 :D-r 500 :K-r 400)
     (send *ri* :start-impedance limb :M-p 0 :D-p 4000 :K-p 5000 :moment-gain #(1 1 1) :M-r 0 :D-r 500 :K-r 600) ;; M = 0 temporarily for impedance follow-up
     )
    (t
     ;; (send *ri* :start-impedance limb :M-p 200 :D-p 4000 :K-p 6000 :moment-gain #(1 1 1) :M-r 50 :D-r 250 :K-r 400) ;; default
     (send *ri* :start-impedance limb :M-p 0 :D-p 4000 :K-p 6000 :moment-gain #(1 1 1) :M-r 0 :D-r 250 :K-r 400) ;; M = 0 temporarily for impedance follow-up
     )
    )
   )
  ;; (:approach-accel-pedal
  ;;  (&key (tm 10000) (debug-view nil))
  ;;  ;; (send robot :rleg :angle-vector #f(0.271875 0.261 -86.08 62.4348 2.384 -0.234 -0.384)) ;; adjusted joint-angles in real hrp2jsknt
  ;;  ;; (send robot :rleg :angle-vector #f(0.269905 0.257049 -84.7676 56.5701 -0.447684 -0.229449 -0.384)) ;; adjusted joint-angles in real hrp2jsknt for new seat (testbed version)
  ;;  ;; (send robot :rleg :angle-vector #f(0.274637 0.265758 -88.9665 55.0 -2.68319 -0.239387 -0.384)) ;; adjusted joint-angles in real hrp2jsknt for new seat (testbed version)
  ;;  (send robot :rleg :angle-vector #f(0.277593 0.269639 -91.1699 48.9728 5.54723 -0.244199 -0.384))
  ;;  (when real
  ;;    (send *ri* :angle-vector (send robot :angle-vector) tm)
  ;;    (send *ri* :wait-interpolation)
  ;;    )
  ;;  )
  ;; (:approach-brake-pedal
  ;;  (&key (tm 10000) (debug-view nil))
  ;;  ;; (send robot :lleg :angle-vector #f(0.0375 -0.5235 -83.014 80.541 9.842 0.6048 -0.564)) ;; adjusted joint-angles in real hrp2jsknt
  ;;  (send robot :lleg :angle-vector #f(0.398437 -1.4895 -97.47 76.7502 20.0 1.5876 -1.548))
  ;;  (when real
  ;;    (send *ri* :angle-vector (send robot :angle-vector) tm)
  ;;    (send *ri* :wait-interpolation)
  ;;    (send self :set-impedance-for-support :lleg)
  ;;    )
  ;;  )
  (:approach-pedal
   (limb target-pedal &key (tm 2500) (debug-view nil))
   (let (approach-result)
     (case target-pedal
       (:accel-pedal
        ;; (send robot :rleg :angle-vector #f(0.277593 0.269639 -91.1699 48.9728 5.54723 -0.244199 -0.384))
        ;; (send robot :rleg :angle-vector #f(-8.96161 -11.923 -84.9409 37.2658 10.0798 15.014 -0.384))
        ;; (send robot :rleg :angle-vector #f(-10.0 0.269639 -91.1699 48.9728 5.54723 -0.244199 -0.384)) ;; adjusted in real vehicle
        ;; (send robot :rleg :angle-vector #f(-9.79801 0.253958 -109.928 60.3974 12.8831 -0.218503 0.18)) ;; adjusted in drive simulator by reach-until-touch
        ;; (send robot :rleg :angle-vector #f(-9.04768 1.17199 -100.84 58.9158 10.7088 -1.14559 -0.108)) ;; adjusted in real polaris in lasvegas
        (send robot :rleg :angle-vector #f(-5.32243 7.27429 -102.357 61.9889 8.87751 -8.28873 -0.108))
        (setq approach-result t)
        )
       (:brake-pedal
        ;; (send robot :lleg :angle-vector #f(0.398437 -1.4895 -97.47 76.7502 20.0 1.5876 -1.548))
        (warn "Approach brake is disabled in hrp2jsk~%")
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
  
  (:step-accel-pedal
   (move-mm &key (time 1000) (relative t) (use-controller t))
   (send robot :rleg :move-end-pos (float-vector 0 0 move-mm) :local) ;; only relative is supported
   (when real
     (if use-controller
         (send *ri* :angle-vector (send robot :angle-vector) time :rleg-controller)
       (send *ri* :angle-vector (send robot :angle-vector) time)
       )
     (send *ri* :wait-interpolation)
     )
   )

  ;; temporary: use kojima bar
  ;; (:approach-frame
  ;;  (&rest args &key (limb :rarm) (target-handle :rear-support-frame-attachment) (offset #f(-150 -100 50))
  ;;         &allow-other-keys)
  ;;  (send-super* :approach-frame :limb limb :target-handle target-handle :offset offset args)
  ;;  )
  ;; temporary: use nackle attachment
  ;; (:approach-frame
  ;;  (&rest args &key (limb :rarm) (target-handle :seat-center) (offset #f(-150 100 0)) (rotation-axis t)
  ;;         &allow-other-keys)
  ;;  (send-super* :approach-frame :limb limb :target-handle target-handle
  ;;               :offset offset :rotation-axis rotation-axis args)
  ;;  )

  ;; for egress
  (:egress-pose
   ()
   (send robot :angle-vector
         #f(-2.794558e-05 0.000354 -91.9067 91.8755 0.031103 -0.000377 0.0 -7.09676 0.281819 -91.8704 92.4525 -0.582257 -0.281853 0.0 0.0 0.0 0.0 0.0 -112.8 -6.8 0.0 -70.28 -14.8452 -11.9705 0.008544 0.0 -9.6 6.8 0.0 -136.0 14.8452 11.9705 0.008544 0.0)
         )
   )
  )

;; (setq *motion* (instance hrp2jsk-driving-motion-on-polaris-xp900 :init :real nil))

