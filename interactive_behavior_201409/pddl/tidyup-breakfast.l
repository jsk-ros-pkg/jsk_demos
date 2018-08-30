#!/usr/bin/env roseus
;; tidyup-breakfast.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")
(require :new-room-domain "package://interactive_behavior_201409/pddl/new-room-domain.l")

(defun plan (&optional debug)
  (setq *domain* (make-domain))
  (setq *problem*
        (make-problem :init '((ROBOT-AT START)
                              (ON BOTTLE TABLE)
                              (ON BOWL TABLE)
                              (ON CUP TABLE)
                              (ON SERIAL TABLE)
                              (ON CLEANER DRAWER)
                              (ON SPOON TABLE))
                      :goal '((ON BOTTLE FRIDGE)
                              (ON BOWL DISHWASHER)
                              (ON CUP KITCHEN)
                              (ON SPOON COUNTER)
                              (ON SERIAL KITCHEN)
                              ) ;; this is from ambiguous
                      :items '(BOTTLE BOWL CUP SPOON SERIAL)
                      :people '(INABA)
                      ))
  (setq *sm* (plan-task *domain* *problem* :debug debug)))

(defun run (&optional debug)
  (plan debug)
  (pprint (exec-preemptive-task *sm*)))

(pr2-init)
