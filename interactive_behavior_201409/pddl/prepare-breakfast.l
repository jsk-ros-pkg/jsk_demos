#!/usr/bin/env roseus
;; deliver-bottle.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")
(require :new-room-domain "package://interactive_behavior_201409/pddl/new-room-domain.l")

(defun plan (&optional debug)
  (setq *domain* (make-domain))
  (setq *problem* (make-problem :init
                                '((ROBOT-AT START)
                                  )
                                :goal
                                '((ON BOWL TABLE)
                                  (ON CUP TABLE)
                                  (ON BOTTLE TABLE))
                                :items
                                '(CUP BOTTLE BOWL BOOK)
                                ))
  (setq *sm* (plan-task *domain* *problem* :debug debug)))

(defun run (&optional debug)
  (plan debug)
  (pprint (exec-preemptive-task *sm*)))

(pr2-init)
