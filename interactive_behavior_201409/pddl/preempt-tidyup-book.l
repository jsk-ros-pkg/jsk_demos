#!/usr/bin/env roseus
;; preempt-tidyup-book.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")

(pr2-init)

(defun run ()
  (send *ri* :preempt)

  (unix:sleep 5)

  (setq init-state (send *ri* :preempted-state))
  (setq goal-state (send *ri* :preempted-goal))

  (push '(on book table) init-state)
  (push '(on book bookshelf) goal-state)

  (send *ri* :preempted-state init-state)
  (send *ri* :preempted-goal goal-state)
  )
