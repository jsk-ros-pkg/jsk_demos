#!/usr/bin/env roseus
;; bring-bottle.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-actionlib "package://interactive_behavior_201409/pddl/pr2-actionlib.l")
(require :new-room-domain "package://interactive_behavior_201409/pddl/new-room-domain.l")

(defun plan (&optional debug)
  (setq *domain* (make-domain))
  (setq *problem* (make-problem :init
                                '((ROBOT-AT START)
                                  (ON BOTTLE FRIDGE)
                                  )
                                :goal
                                '((ON BOTTLE TABLE))
                                :items
                                '(BOTTLE)
                                ))
  (setq *sm* (plan-task *domain* *problem* :debug debug :use-cache nil)))
#|
(defun cont ()
  (ros::ros-info "mydata: ~A" *mydata*)
  (close-door 'rarm 'fridge *mydata*)
  (move-to 'fridge 'table *mydata*)
  (find-placement 'bottle larm table *mydata*)
  (place 'bottle 'larm 'table))
|#


(defun run (&optional debug)
  (plan debug)
  (setq *mydata* '(nil))
  (pprint (exec-preemptive-task *sm* :mydata *mydata*))

  (ros::rate 1)
  (while (and (ros::ok) (preempted-p))
    (ros::ros-info "waiting for recovery")
    (ros::sleep))
  (ros::ros-info "continue")
  (setq *from-state* (send *sm* :node "(close-door rarm fridge)"))
  (exec-preemptive-task *sm*
                        :mydata *mydata*
                        :from-state *from-state*)
)

(pr2-init)
