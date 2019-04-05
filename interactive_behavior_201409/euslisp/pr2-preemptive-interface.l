#!/usr/bin/env roseus
;; pr2-preemptive-interface.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(provide :pr2-preemptive-interface)

(require :preemptlib "package://interactive_behavior_201409/euslisp/preemptlib.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

;; suppress print violate max/min-angle
(unless (assoc :joint-angle-org (send rotational-joint :methods))
  (rplaca (assoc :joint-angle (send rotational-joint :methods)) :joint-angle-org)
  (defmethod rotational-joint
      (:joint-angle
       (&optional v &key relative &allow-other-keys)
       (let ()
         (when v
           (when (and joint-min-max-table joint-min-max-target)
             (setq min-angle (send self :joint-min-max-table-min-angle)
                   max-angle (send self :joint-min-max-table-max-angle)))
           (if relative (setq v (+ v joint-angle)))
           (cond ((> v max-angle)
                  (setq v max-angle)))
           (cond ((< v min-angle)
                  (setq v min-angle)))
           (setq joint-angle v)
           (send child-link :replace-coords default-coords)
           (send child-link :rotate (deg2rad joint-angle) axis))
         joint-angle)))
  ) ;; defmethod rotational-joint
(unless (assoc :joint-angle-org (send linear-joint :methods))
  (rplaca (assoc :joint-angle (send linear-joint :methods)) :joint-angle-org)
  (defmethod linear-joint
      (:joint-angle
       (&optional v &key relative &allow-other-keys)
       "return joint-angle if v is not set, if v is given, set joint angle. v is linear value in [mm]."
       (let ()
         (when v
           (if relative (setq v (+ v joint-angle)))
           (when (> v max-angle)
             (setq v max-angle))
           (when (< v min-angle)
             (setq v min-angle))
           (setq joint-angle v)
           (send child-link :replace-coords default-coords)
           (send child-link :translate (scale joint-angle axis)))
         joint-angle)))
  ) ;; defmethod linear-joint

;; defclass pr2-preempt-interface
(defpreemptrobot pr2
  (send r-gripper-action :cancel-all-goals)
  (send l-gripper-action :cancel-all-goals)
  (send self :cancel-angle-vector)
  (send self :go-stop nil))
