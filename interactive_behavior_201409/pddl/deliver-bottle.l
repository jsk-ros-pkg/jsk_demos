#!/usr/bin/env roseus
;; deliver-bottle.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :preemptlib "package://interactive_behavior_201409/euslisp/preemptlib.l")
(require :pr2-actionlib "package://interactive_behavior_201409/euslisp/pr2-actionlib.l")
(require :tidyup-domain "package://interactive_behavior_201409/pddl/tidyup-domain.l")

(defun plan (&optional debug)
  (setq *domain* (make-tidyup-domain))
  (setq *problem*
        (make-problem :init '((AT START)
                              (ON BOTTLE FRIDGE)
                              (= (total-cost) 0))
                      :goal '((DELIVERED BOTTLE 73A2) (AT START))
                      ))
  (setq *sm* (plan-task *domain* *problem* :debug debug)))

(defun run (&optional debug)
  (plan debug)
  (pprint (exec-preemptive-task *sm*)))

(pr2-init)
