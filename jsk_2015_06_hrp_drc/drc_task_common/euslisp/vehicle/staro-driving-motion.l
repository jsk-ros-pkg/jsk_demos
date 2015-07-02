#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/robot-driving-motion.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/staro-interface.l")
(load "package://drc_task_common/euslisp/vehicle/drive-simulator-environment-ranger-xp900-size.l")

(defclass staro-driving-motion
  :super robot-driving-motion
  )

(defmethod staro-driving-motion
  (:init
   (&key ((:car tmp-car) nil) ((:real tmp-real) nil) (grasp-arm :rarm) (grasp-point :handle-top))
   (staro-init)
   (unless tmp-car
     (setq tmp-car (instance drive-simulator-ranger-xp900-size :init :handle-mode :left))
     )
   (send-super :init :robot *staro* :car tmp-car :real tmp-real :grasp-arm grasp-arm :grasp-point grasp-point)
   )  

  ;; initial methods
  (:drive-init-pose
   (&key (tm 20000))
   (send robot :reset-manip-pose)
   (case (send self :get :grasp-arm)
     (:rarm nil
      ;; (send robot :larm :angle-vector #f(0.0 0.0 0.0 -90.0 90.0 0.0 0.0 -45.0)) ;; open larm to avoid handle
      )
     (:larm nil) ;; TODO: find out and implement init-pose
     )
   (send robot :legs :angle-vector #f(0.0 0.0 -70.0 64.0 6.5 0.0))
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )
   (send *ri* :hand :arms :close)
   )
  (:drive-init-pose-before-sitting
   (&key (tm 20000))
   (case (send self :get :grasp-arm)
     (:rarm
      ;; (send robot :larm :angle-vector #f(0.0 0.0 0.0 -90.0 90.0 0.0 0.0 -45.0)) ;; bend larm to get into drive simulator
      
      ;; (send robot :rarm :angle-vector #f(87.0121 31.6621 49.0507 147.317 -134.808 -173.1 -82.6094 23.7911)) ;; approach-handle + #f(-200 0 0)
      ;; (send robot :rarm :angle-vector #f(47.9289 120.621 -66.0896 80.6666 -122.449 54.3689 53.3458 63.5505))
      ;; (send robot :rarm :angle-vector #f(52.3259 130.463 -101.817 59.2213 -98.5639 8.91942 48.119 95.5304))
      ;; (send robot :rarm :angle-vector #f(49.3246 111.383 -103.494 50.5975 -88.8747 -1.19363 23.7759 94.7362)) ;; + #f(0 0 150)

      ;; elbow is upper than handle
      ;; (send robot :rarm :angle-vector #f(45.7577 129.158 -112.445 53.1436 -92.4657 3.56696 45.7696 95.6754))
      (send robot :rarm :angle-vector #f(49.3246 111.383 -103.494 50.5975 -88.8747 -1.19363 23.7759 94.7362)) ;; elbow is more higher 
      ;; elbow is lower than handle but can solve ik more
      ;; (send robot :rarm :angle-vector #f(90.0099 53.6678 92.1434 98.7068 -134.999 8.36409 81.8336 -79.666))
      )
     (:larm
      ;; (send robot :larm :angle-vector #f(-51.904 -107.301 33.9072 -139.648 129.044 74.915 72.5736 68.4857))
      ;; (send robot :larm :angle-vector #f(-60.3382 -179.897 -49.1277 -34.6274 132.082 9.9844 39.0322 117.241))
      ;; (send robot :larm :angle-vector #f(-32.4584 -179.714 12.1032 -27.7713 113.882 39.0826 30.986 103.93))
      ;; (send robot :larm :angle-vector #f(9.61778 -179.861 85.2553 -31.3058 54.9522 36.599 57.6563 128.973))
      (send robot :larm :angle-vector #f(-6.58151 -50.0866 -44.5613 7.11083 -112.932 -1.02229 -18.0822 -65.1661)) ;; avoid collision with handle
      ) ;; TODO: find out and implement init-pose
     )
   
   (send robot :lleg :angle-vector #f(0.0 0.0 -85 55.0 -40.0 0.0))
   (send robot :rleg :angle-vector #f(0.0 0.0 -85 55.0 -40.0 0.0))
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )
   )
  (:drive-init-pose-after-sitting
   (&key (tm 20000))
   (case (send self :get :grasp-arm)
     (:rarm
      ;; (send robot :larm :angle-vector #f(0.0 0.0 90.0 -90.0 90.0 0.0 0.0 -45.0)) ;; bend larm to improve cog in drive simulator
      )
     (:larm
      ;; (send robot :larm :angle-vector #f(9.61778 -179.861 85.2553 -31.3058 54.9522 36.599 57.6563 128.973)) ;; bending outside
      ;; (send robot :larm :angle-vector #f(-8.0784 22.5284 -77.1556 -69.6657 -84.2627 -98.2629 67.3799 99.628)) ;; elbow is lower
      ;; (send robot :larm :angle-vector #f(-24.3437 -32.5517 -25.2969 14.4575 -105.139 4.86937 -27.4592 -45.4563)) ;; elbow is middle
      )
     )
   (send robot :lleg :angle-vector #f(0.0 0.0 -70 55.0 -40.0 0.0))
   (send robot :rleg :angle-vector #f(0.0 0.0 -70 55.0 -40.0 0.0))
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) tm)
     (send *ri* :wait-interpolation)
     )
   )
  (:ride
   (&key (init-pose t))
   (send robot :move-to (make-coords :pos (float-vector -25 -55.0 270) :rpy (float-vector 0.0 0.0 0.0)) :world)
   (when init-pose
     (send self :drive-init-pose)
     )
   )

  ;; for robotiq hand
  (:open-hand
   (&optional (limb (send self :grasp-arm)))
   (when real
     (send *ri* :hand limb :open)
     (send *ri* :hand limb :wait)
     )
   )
  (:close-hand
   (&optional (limb (send self :grasp-arm)))
   (when real
     (send *ri* :hand limb :close)
     (send *ri* :hand limb :wait)
     )
   )

  ;; schaft sttering
  (:schaft-approach-handle
   (&key (limb (send self :get :grasp-arm)) (offset #f(-80 0 0))
         (tm 15000) (sync-controller nil) (debug-view nil) (start-impedance t) (stop-impedance t)
         (null-space #f(-0.06 -0.3 0.0 -0.18 0.16 0.0 0.0 0.0))
         )
   (when start-impedance
     (send *ri* :start-impedance limb
           :M-p 100 :D-p 300 :K-p 700 :M-r 30 :D-r 80 :K-r 200
           )
     )
   (send self :approach-handle
         :limb limb :target-handle :handle-center
         :tm tm :sync-controller sync-controller :debug-view debug-view :offset offset
         :start-impedance nil :stop-impedance stop-impedance :null-space null-space
         )
   (send *ri* :hand limb :wide-mode)
   (send *ri* :hand limb :wait)
   )
  (:schaft-grasp-handle
   (&key (limb (send self :get :grasp-arm))
         (debug nil) (tm 10000) (debug-view nil) (start-impedance t))
   (when start-impedance
     (send *ri* :start-impedance limb
           :M-p 100 :D-p 300 :K-p 700 :M-r 30 :D-r 80 :K-r 200)
     )
   (send self :grasp-handle :limb limb :target-handle :handle-center
         :debug debug :tm tm :debug-view debug-view :start-impedance nil)
   )
  (:schaft-turn-handle
   (deg &key (limb (send self :get :grasp-arm)) ;; grasp-point is assumed to be handle-center
        (time 300) (relative t) (debug-view nil) (use-controller t))
   (let ((target-degree (if relative deg (- deg (send car :handle-joint :joint-angle))))
         (target-robot-joint (send robot limb :wrist-p))
         real-target-degree
         )
     (setq real-target-degree (max (max (- (send target-robot-joint :min-angle) (send target-robot-joint :joint-angle))
                                        (- (send (send car :handle-joint) :min-angle) (send (send car :handle-joint) :joint-angle)))
                                   (min (min (- (send target-robot-joint :max-angle) (send target-robot-joint :joint-angle))
                                             (- (send (send car :handle-joint) :max-angle) (send (send car :handle-joint) :joint-angle)))
                                        target-degree)
                                   )
           )
     (send target-robot-joint :joint-angle real-target-degree :relative t)
     (send car :handle-joint :joint-angle real-target-degree :relative t)
     (when real
       ;; define controller
       (if use-controller
           (case limb
             (:larm (setq controller :larm-controller))
             (:rarm (setq controller :rarm-controller))
             (:lleg (setq controller :lleg-controller))
             (:rleg (setq controller :rleg-controller))
             (t (setq controller :default-controller))
             )
         (setq controller :default-controller)
         )
       (send *ri* :angle-vector (send robot :angle-vector) (abs (* time real-target-degree)) controller)
       (send *ri* :wait-interpolation)
       )
     )
   )

  ;; holding car frame
  (:approach-frame
   (&key (limb :rarm) (target-handle :seil-left) (time 10000) (use-controller t) (offset #f(100 0 250)) (debug-view nil))
   (send self :open-hand :rarm)
   (send robot :rarm :angle-vector #f(-6.46278 93.9017 69.7888 -2.8307 -92.3049 -4.90419 -66.9425 -52.3743))
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) time)
     (send *ri* :wait-interpolation)
     ;; (send *viewer* :draw-objects)
     )
   )
  (:grasp-frame
   (&key (limb :rarm) (target-handle :seil-left) (time 5000) (use-controller t) (offset #f(0 0 250)) (debug-view nil))
   (send robot :rarm :angle-vector #f(-9.42783 97.3642 24.7373 -7.26613 -92.4082 -6.29007 -21.6825 -51.7752))
   (when real
     (send *ri* :angle-vector (send robot :angle-vector) time)
     (send *ri* :wait-interpolation)
     ;; (send *viewer* :draw-objects)
     )
   (send self :close-hand :rarm)
   )

  ;; for sitting
  (:make-force-sitting-motion
   (&key (drive-init-pose nil))
   ;; initialize
   (let ((force-sitting-avs-and-msg nil))
     ;; make drive-init-pose
     (send robot :legs :angle-vector #f(0.0 0.0 -70.0 64.0 6.5 0.0))
     (when drive-init-pose
       (push (list (send robot :angle-vector) "drive-init-pose") force-sitting-avs-and-msg)
       )
     ;; lift up legs
     (send robot :legs :move-end-pos (float-vector 0 0 250)) ;; lift up legs
     (push (list (send robot :angle-vector) "lift-up-legs 250") force-sitting-avs-and-msg)
     ;; avoid legs from seat 
     (send robot :legs :ankle-p :joint-angle -30 :relative t)
     (send robot :legs :knee-p :joint-angle -20 :relative t)
     (push (list (send robot :angle-vector) "avoid-legs") force-sitting-avs-and-msg)
     ;; lift down legs
     (dotimes (i 4) 
       (send robot :legs :move-end-pos (float-vector 0 0 -40)) ;; lift down legs
       (push (list (send robot :angle-vector) "lift-down-legs 40") force-sitting-avs-and-msg)
       )
     (send robot :legs :crotch-p :joint-angle 15 :relative t)
     (push (list (send robot :angle-vector) "adjust crotch") force-sitting-avs-and-msg)
     (setq force-sitting-avs-and-msg (reverse force-sitting-avs-and-msg)) ;; set force-sitting-avs to sitting order
     force-sitting-avs-and-msg
     )
   )
  (:force-getting-on-motion
   (&key (interactive t))
   ;; make sitting motion avs
   (let ((avs-msg nil))
     (setq avs-msg (send self :make-force-sitting-motion :drive-init-pose nil))
     (dolist (av-msg avs-msg)
       (send robot :angle-vector (car av-msg))
       (when interactive
         (warn "~A~%" (cadr av-msg))
         (read-line)
         )
       (when real
         (send *ri* :angle-vector (send robot :angle-vector) 10000 :lleg-controller)
         (send *ri* :angle-vector (send robot :angle-vector) 10000 :rleg-controller)
         (send *ri* :wait-interpolation-of-group "lleg")
         (send *ri* :wait-interpolation-of-group "rleg")

         )
       ;; (send *viewer* :draw-objects)
       )
     ;; finally sync controller
     (when real
       (send *ri* :sync-controller :lleg-controller)
       (send *ri* :sync-controller :rleg-controller)
       )
     )
   )
  (:force-getting-off-motion
   (&key (interactive t))
   ;; make sitting motion avs
   (let ((avs-msg nil))
     (setq avs-msg (reverse (send self :make-force-sitting-motion :drive-init-pose t)))
     (dolist (av-msg avs-msg)
       (send robot :angle-vector (car av-msg))
       (when interactive
         (warn "~A~%" (cadr av-msg))
         (read-line)
         )
       (when real
         (send *ri* :angle-vector (send robot :angle-vector) 10000 :lleg-controller)
         (send *ri* :angle-vector (send robot :angle-vector) 10000 :rleg-controller)
         (send *ri* :wait-interpolation-of-group "lleg")
         (send *ri* :wait-interpolation-of-group "rleg")
         )
       ;; (send *viewer* :draw-objects)
       )
     ;; finally sync controller
     (when real
       (send *ri* :sync-controller :lleg-controller)
       (send *ri* :sync-controller :rleg-controller)
       )
     )
   )

  ;; utils
  (:re-grasp
   (limb)
   (send *ri* :hand limb :recovery)
   (send *ri* :hand limb :close)
   )
  (:set-impedance-for-handle
   (limb)
   (send *ri* :start-impedance limb :M-p 200 :D-p 600 :K-p 1400)
   )
  )

;; (setq *motion* (instance staro-driving-motion :init :real nil))
