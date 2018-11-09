#!/usr/bin/env roseus
;; tidyup-tray.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")
(require :new-room-domain "package://interactive_behavior_201409/pddl/new-room-domain.l")

(defun plan (&optional debug)
  (setq *domain* (make-domain :use-arms t))
  (setq *problem* (make-problem :init
                                '((ROBOT-AT START)
                                  (ON TRAY TABLE)
                                  )
                                :goal
                                '((ON TRAY KITCHEN))
                                :items
                                '(TRAY)
                                :use-arms t
                                ))
  (setq *sm* (plan-task *domain* *problem* :debug debug :use-cache nil)))
