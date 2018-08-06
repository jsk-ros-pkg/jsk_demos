#!/usr/bin/env roseus
;; pr2-actionlib.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")
(require :preemptlib "package://interactive_behavior_201409/euslisp/preemptlib.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
;;
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :coe-actionlib "package://jsk_2017_12_pr2_coe/euslisp/coe-actionlib.l")

;; defclass pr2-preempt-interface
(defpreemptrobot pr2
  (send r-gripper-action :cancel-all-goals)
  (send l-gripper-action :cancel-all-goals)
  (send self :cancel-angle-vector)
  (send self :go-stop nil))

(defaction move-to (from-place to-place &optional userdata)
  (ros::ros-info "(move-to ~A ~A)" from-place to-place)
  ;; TODO: record START place
  (save-start-pose)
  (case to-place
    ('kitchen (go-to-sink userdata)) ;; coe
    ('dishwasher (go-to-dishwasher userdata)) ;; coe
    (t
     (let (target-spot)
       (if (cdr (assoc :target-place userdata))
           (setq target-spot (get-spot (cdr (assoc :target-place userdata))))
         (setq target-spot (get-spot to-place)))
       (use-tilt-laser-obstacle-cloud nil)
       (prog1 (send *ri* :move-to target-spot)
         (use-tilt-laser-obstacle-cloud t))))))


(defaction pick (obj arm spot &optional userdata)
  (cond
   ((eq spot 'kitchen) ;; coe
    (find-object-at-kitchen userdata)
    (pick-object-at-kitchen userdata))
   (t t)))

(defaction place (obj arm spot &optional userdata)
  (cond
   ((eq spot 'dishwasher) ;; coe
    (put-into-dishwasher userdata))
   (t t)))

(defaction open-door (arm spot &optional userdata)
  (cond
   ((eq spot 'dishwasher) ;; coe
    (open-dishwasher userdata))
   (t t)))

(defaction close-door (arm spot &optional userdata)
  (cond
   ((eq spot 'dishwasher)
    (close-dishwasher userdata))
   (t t)))

(defaction deliver (obj arm spot &optional userdata)
  ;; TODO
  t)

(defaction ask (spot &optional userdata)
  ;; TODO
  t)


(provide :pr2-actionlib)

#|
(defaction pick (target arm spot &optional userdata)
  (let (target-cds pregrasp grasp)
    ;; FIXME
    (setq target-cds (make-coords :pos (float-vector 800 0 600)))
    ;;
    (setq arm (intern (symbol-string arm) *keyword-package*))
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

(defaction place (target arm spot &optional userdata)
  (let (target-cds preungrasp ungrasp)
    ;; FIXME
    (setq target-cds (make-coords :pos (float-vector 800 0 600)))
    ;;
    (setq arm (intern (symbol-string arm) *keyword-package*))
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

(defaction move-to (from-spot to-spot &optional userdata)
  (let (cds)
    ;; FIXME
    (let ((start-time (ros::time-now)))
      (while (and (ros::ok)
                  (< (send (ros::time- (ros::time-now) start-time) :to-sec) 10.0))
             (send *ri* :ros-wait 1.0 :spin t :spin-self t)))
    (return-from move-to t)
    (setq cds
          (case to-spot
            ('START (make-coords :pos (float-vector 0 0 0)))
            ('TABLE (make-coords :pos (float-vector 0 5000 0)))
            ('KITCHEN (make-coords :pos (float-vector 10000 0 0)))
            ('COUNTER (make-coords :pos (float-vector 5000 0 0)))
            (t (error "unknown spot" to-spot))))
    (send *ri* :move-to cds)))
|#
