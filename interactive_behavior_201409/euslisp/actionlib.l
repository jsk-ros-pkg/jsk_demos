#!/usr/bin/env roseus
;; actionlib.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :preemptlib "package://interactive_behavior_201409/euslisp/preemptlib.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

;; defclass pr2-preempt-interface
(defpreemptrobot pr2
  (send r-gripper-action :cancel-all-goals)
  (send l-gripper-action :cancel-all-goals)
  (send self :cancel-angle-vector)
  (send self :go-stop nil))


(defaction pick (target arm)
  (let ((target-cds target) ;; TODO: loopup coords from object symbol
        pregrasp grasp)
    (setq grasp
     (send *robot* arm :inverse-kinematics target-cds
                       :rotation-axis :z
                       :look-at-target t))
    (setq pregrasp
     (send *robot* arm :move-end-pos
           (float-vector -100 0 0)))
    (unless (and grasp pregrasp)
      (ros::ros-error "ik fail")
      (return-from pick nil))
    (send *ri* :stop-grasp arm :wait t)
    ;;
    (send *ri* :angle-vector-sequence
          (list pregrasp grasp)
          (list 10000 3000))
    (send *ri* :wait-interpolation)
    ;;
    (send *ri* :start-grasp arm)
    ;;
    (send *ri* :angle-vector pregrasp 3000)
    (send *ri* :wait-interpolation)))

(defaction place (target arm)
  (let ((target-cds target)
        preungrasp ungrasp)
    (setq ungrasp
          (send *robot* arm :inverse-kinematics target-cds
                            :rotation-axis :z
                            :look-at-target t))
    (setq preungrasp
          (send *robot* arm :move-end-pos
                (float-vector 0 0 100)))
    (unless (and preungrasp ungrasp)
      (ros::ros-error "ik fail")
      (return-from place nil))
    ;;
    (send *ri* :angle-vector-sequence
          (list preungrasp ungrasp)
          (list 10000 3000))
    (send *ri* :wait-interpolation)
    ;;
    (send *ri* :stop-grasp arm :wait t)
    ;;
    (send *ri* :angle-vector preungrasp 3000)
    (send *ri* :wait-interpolation)))

(defaction move-to (from-spot to-spot)
  (let ((to-cds to-spot)) ;; lookup cds
    (send *ri* :move-to to-cds)))
