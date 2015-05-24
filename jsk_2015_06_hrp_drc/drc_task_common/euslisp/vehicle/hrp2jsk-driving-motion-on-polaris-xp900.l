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
   ;; (send robot :move-to (make-coords :pos (float-vector -140 420.0 180) :rpy (float-vector 0.0 (deg2rad 5.0) 0.0)) :world)
   ;; (send robot :move-to (make-coords :pos (float-vector -80 470.0 130) :rpy (float-vector 0.0 0.0 0.0)) :world)
   (send robot :move-to (make-coords :pos (float-vector -80 450.0 130) :rpy (float-vector 0.0 0.0 0.0)) :world)
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

   ;; (send robot :reset-manip-pose) ;; adjusted in real polaris in lasvegas

   ;; (send robot :lleg :angle-vector #f(4.8375 -1.095 -88.646 72.6426 10.4 1.0224 0.216))
   ;; (send robot :rleg :angle-vector #f(-9.04768 1.17199 -100.84 58.9158 10.7088 -1.14559 -0.108))
   ;; (send robot :rarm :angle-vector #f(35.9754 -42.1297 8.69405 -74.6325 -43.4673 3.3033 35.8167 0.0))

   ;; (send robot :lleg :angle-vector #f(5.66017 7.49564 -86.8436 69.5621 12.0391 -7.95977 0.216))
   ;; (send robot :rleg :angle-vector #f(-5.32243 7.27429 -102.357 61.9889 8.87751 -8.28873 -0.108))
   ;; (send robot :rarm :angle-vector #f(35.9754 -42.1297 8.69405 -74.6325 -43.4673 3.3033 35.8167 0.0))

   ;; (send robot :angle-vector #f(0.000746 -0.012134 -94.3573 94.1927 0.163626 0.012907 0.0 -7.09667 0.305781 -94.3208 94.7708 -0.450903 -0.306558 0.0 0.0 0.0 0.0 0.0 -146.447 3.83694 11.5024 -41.098 -24.6756 -7.84617 -5.18264 27.1571 16.4364 8.84719 36.8944 -135.246 1.71672 -14.8073 -3.26523 -27.3191)) ;; noda-pose
   ;; (send robot :angle-vector #f(-16.1353 -10.3899 -96.856 61.3548 3.71207 12.3977 -0.108 -7.08957 0.294294 -93.7843 94.2707 -0.486473 -0.294315 0.0 0.0 0.0 0.0 0.0 22.7148 -49.6094 35.9183 -66.4958 -58.3108 -2.56987 25.3079 60.0 -37.8397 15.3281 -24.3301 -102.858 41.5248 -59.8958 27.1202 -27.3191)) ;; adjusted on polaris
   ;; (send robot :angle-vector #f(-9.0486 1.17029 -101.857 71.8007 -1.15964 -1.1434 -0.108 -7.09068 19.4052 -88.1697 91.111 -2.94105 -19.4056 0.0 0.0 0.0 0.0 0.0 50.0 -30.0 -10.0 -120.0 -25.0 -5.0 -20.0 60.0 -37.8397 15.3281 -24.3301 -102.858 41.5248 -59.8958 27.1202 -27.3191)) ;; move -100 above 
   (send robot :angle-vector #f(-12.4445 -4.72067 -99.3983 66.7089 0.989975 5.88808 0.252 -7.2949 19.9406 -89.6837 92.3316 -2.63926 -20.0881 -0.18 -0.052 0.104734 -6.54258 27.3164 22.6578 -49.6353 35.6875 -66.5803 -58.4068 -2.95173 25.3078 0.0 -35.1093 36.6454 -33.3234 -103.991 22.1522 -57.618 -14.0548 0.0)) ;; move -50 above

   (send robot :rarm :angle-vector #f(50.0 -30.0 -10.0 -120.0 -25.0 -5.0 -20.0 60.0)) ;; reset-mainip-pose
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
   (&rest args &key (offset #f(-100 50 100)) &allow-other-keys)
   (send-super* :approach-handle :offset offset args)
   )
  (:grasp-handle
   (&rest args &key (offset #f(-45 -10 0)) &allow-other-keys)
   (send-super* :grasp-handle :offset offset args)
   )
  (:turn-handle-once
   (deg &rest args &key (offset #f(-45 -10 0)) &allow-other-keys)
   (send-super* :turn-handle-once deg :offset offset args)
   )
  (:approach-floor
   (&rest args &key (target-handle :floor-footrest) &allow-other-keys)
   (send-super* :approach-floor :target-handle target-handle args)
   )
  (:estimate-current-handle-angle
   (&rest args &key (offset (scale -1 #f(-45 -10 0))) &allow-other-keys)
   (send-super* :estimate-current-handle-angle :offset offset args)
   )
  (:set-impedance-for-handle
   (limb)
   ;; (send *ri* :start-impedance limb :moment-gain #f(0 0 0) :D-p 400 :K-p 800)
   ;; (send *ri* :start-impedance limb :moment-gain #f(0 0 0) :M-p 0.0 :D-p 400 :K-p 800) ;; M = 0 temporarily for impedance follow-up
   (send *ri* :start-impedance limb :M-p 0.0 :D-p 800 :K-p 1200
         :moment-gain #f(1 1 1) :M-r 0.0 :D-r 300 :K-r 400
         ) ;; M = 0 temporarily for impedance follow-up
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
        ;; (send robot :rleg :angle-vector #f(-5.32243 7.27429 -102.357 61.9889 8.87751 -8.28873 -0.108)) ;; noda-pose
        ;; (send robot :rleg :angle-vector #f(-16.1353 -10.3899 -96.856 61.3548 3.71207 12.3977 -0.108)) ;; adjusted on real polaris
        ;; (send robot :rleg :angle-vector #f(-9.0486 1.17029 -101.857 71.8007 -1.15964 -1.1434 -0.108)) ;; move -100 above
        (send robot :rleg :angle-vector #f(-12.4444 -4.72054 -99.3971 66.7074 0.99031 5.88795 0.252)) ;; move -50 above
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

;; angle-vector lleg 10 up to :z
;; #f(-16.1353 -10.3899 -96.856 61.3548 3.71207 12.3977 -0.108 -7.08215 0.29671 -95.7005 96.0273 -0.326797 -0.296752 0.0 0.0 0.0 0.0 0.0 22.7073 -49.7459 36.0952 -66.6313 -58.4738 -2.68813 25.2444 60.0 -38.3715 16.4109 -25.431 -102.848 41.1468 -59.0094 25.5658 -27.3191)
;; (send *hrp2jsknts* :angle-vector)
;; move :legs 50 to y and :lleg 5 to :z
;; #f(-16.1351 -10.3895 -96.8545 61.3529 3.71253 12.3973 -0.108 -7.08201 10.6506 -93.6542 95.2857 -1.63162 -10.6506 0.0 0.0 0.0 -6.6 27.2 22.7073 -49.7459 36.0952 -66.6313 -58.4738 -2.68813 25.2444 60.0 -45.7106 15.7603 -57.7535 -97.3731 19.889 -35.0751 -3.41394 -27.3191)
;; move :legs 50 more
;; #f(-12.4445 -4.72067 -99.3983 66.7089 0.989975 5.88808 0.252 -7.2949 19.9406 -89.6837 92.3316 -2.63926 -20.0881 -0.18 -0.052 0.104734 -6.54258 27.3164 22.6578 -49.6353 35.6875 -66.5803 -58.4068 -2.95173 25.3078 0.0 -35.1093 36.6454 -33.3234 -103.991 22.1522 -57.618 -14.0548 0.0)

;; send *ri* :start-impedance :larm :M-p 0.0 :D-p 800 :K-p 1600 :moment-gain #f(1 1 1) :M-r 0.0 :D-r 20 :K-r 30
;; #f(-12.4444 -4.72054 -99.3971 66.7074 0.99031 5.88795 0.252 -7.2949 19.9406 -89.6837 92.3316 -2.63926 -20.0881 -0.18 -0.052 0.104734 -6.54258 27.3164 22.6578 -49.6353 35.6875 -66.5803 -58.4068 -2.95173 25.3078 0.0 -39.6277 21.9582 -43.2871 -102.884 23.7294 -47.6582 0.203336 0.0)
;; grasp offset 
;; 0.015 0.07 0
;; args="-0.145 0.07 0 0 0 0 handle_pose crank_pose"
