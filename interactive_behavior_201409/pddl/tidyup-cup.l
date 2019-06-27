#!/usr/bin/env roseus
;; tidyup-cup.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")
(require :room-domain "package://interactive_behavior_201409/pddl/room-domain.l")

(defun plan (&optional debug)
  (setq *domain* (make-domain))
  (setq *problem*
        (make-problem :init '((AT START)
                              (ON CUP COUNTER)
                              (= (total-cost) 0))
                      :goal '((OR (ON CUP KITCHEN) (ON CUP DISHWASHER)))
                      :items '(CUP)
                      ))
  (setq *sm* (plan-task *domain* *problem* :debug debug))
  (setq *insp* (instance state-machine-inspector :init *sm*))
  (send *sm* :reset-state)
  (send *insp* :publish-structure)
  (send *insp* :publish-all-status)
  *sm*)

(defun run (&optional debug)
  (plan debug)
  (pprint (exec-preemptive-task *sm*)))

(pr2-init)
