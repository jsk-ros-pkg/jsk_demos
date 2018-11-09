#!/usr/bin/env roseus
;; tidyup-cup.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")
(require :room-domain "package://interactive_behavior_201409/pddl/new-room-domain.l")

(defun plan (&optional debug)
  (setq *domain* (make-domain))
  (setq *problem*
        (make-problem :init '((ROBOT-AT TABLE)
                              (ON CUP TABLE))
                      :goal '((ON CUP KITCHEN))
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
  (setq *mydata* '(nil))
  (pprint (exec-preemptive-task *sm* :mydata *mydata*)))


(defun run-continue ()
  (run)
  (ros::rate 1)
  (while (and (ros::ok) (preempted-p))
    (ros::ros-info "waiting for recovery")
    (ros::spin-once)
    (ros::sleep))
  (ros::ros-info "continue")
  (setq *from-state* (send *sm* :node "(move-to table kitchen)"))
  (exec-preemptive-task *sm*
                        :mydata *mydata*
                        :from-state *from-state*))

(pr2-init)
