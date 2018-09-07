#!/usr/bin/env roseus
;; tidyup-bottle.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")
(require :problem-parser "package://mbtk/euslisp/problem-parser.l")

(defun plan (&optional debug)
  (setq *domain* (make-domain))
  (setq *problem*
        (make-problem :init '((ROBOT-AT START)
                              (ON BOTTLE TABLE))
                      :goal '((ROBOT-AT START)
                              (ON BOTTLE FRIDGE))
                      :items '(BOTTLE)))
  (resolve-usual *problem*)
  (setq *sm* (plan-task *domain* *problem* :debug debug :use-cache t :timeout 30)))

(defun run (&optional debug)
  (plan debug)
  (pprint (exec-preemptive-task *sm*)))

(pr2-init)
