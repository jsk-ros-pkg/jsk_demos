#!/usr/bin/env roseus
;; prepare-picnic.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")
(require :room-domain "package://interactive_behavior_201409/pddl/room-domain.l")

(defun plan (&optional debug)
  (setq *domain* (make-domain))
  (setq *problem*
        (make-problem :init '((AT START)
                              (ON APPLE FRIDGE)
                              (ON BACKPACK KITCHEN)
                              (ON PHONE COUNTER)
                              (= (total-cost) 0))
                      :goal '((ON APPLE TABLE)
                              (ON BACKPACK TABLE)
                              (ON PHONE TABLE))
                      :items '(APPLE BACKPACK PHONE)
                      ))
  (setq *sm* (plan-task *domain* *problem* :debug debug)))

(defun run (&optional debug)
  (plan debug)
  (pprint (exec-preemptive-task *sm*)))

(pr2-init)
