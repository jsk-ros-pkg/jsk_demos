#!/usr/bin/env roseus
;; tidyup-breakfast.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "tidyup_breakfast")

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")
(require :new-room-domain "package://interactive_behavior_201409/pddl/new-room-domain.l")
(require :problem-parser "package://mbtk/euslisp/problem-parser.l")

(defun plan (&optional debug)
  (setq *domain* (make-domain))
  (setq *problem*
        (make-problem :init '((ROBOT-AT START)
                              (ON BOTTLE TABLE)
                              (ON BOWL TABLE)
                              (ON CUP TABLE)
                              ;; (ON SERIAL TABLE)
                              ;; (ON CLEANER DRAWER)
                              ;; (ON SPOON TABLE))
                              )
                      :goal '((ON BOTTLE USUAL)
                              (ON BOWL USUAL)
                              (ON CUP USUAL)
                              ;; (ON SPOON COUNTER)
                              ;; (ON SERIAL KITCHEN)
                              ) ;; this is from ambiguous
                      :items '(BOTTLE BOWL CUP SPOON SERIAL)
                      ))
  (resolve-usual *problem*)
  ;; (pprint (send *problem* :goal-condition))
  (setq *sm* (plan-task *domain* *problem* :debug debug :use-cache t :timeout 30))
)

(defun run (&optional debug)
  (plan debug)
  (when (ros::wait-for-service "/idle_behavior/disable" 3)
    (call-empty-service "/idle_behavior/disable"))
  (pprint (exec-preemptive-task *sm*)))

(pr2-init)
