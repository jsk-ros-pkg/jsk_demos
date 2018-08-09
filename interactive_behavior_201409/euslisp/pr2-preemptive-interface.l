#!/usr/bin/env roseus
;; pr2-preemptive-interface.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(provide :pr2-preemptive-interface)

(require :preemptlib "package://interactive_behavior_201409/euslisp/preemptlib.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

;; defclass pr2-preempt-interface
(defpreemptrobot pr2
  (send r-gripper-action :cancel-all-goals)
  (send l-gripper-action :cancel-all-goals)
  (send self :cancel-angle-vector)
  (send self :go-stop nil))
