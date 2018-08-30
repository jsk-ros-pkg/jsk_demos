#!/usr/bin/env roseus
;; place.l

(ros::roseus "run_place")

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")

(defun plan-place (object place)
  (setq *domain* (make-domain))
  (setq *problem* (make-problem :init '((ROBOT-AT START))
                                :goal `((ON ,object ,place))
                                :items `(,object)))
  (unless (setq *sm* (plan-task *domain* *problem* :debug t))
    (ros::ros-error "failed to plan")
    (return-from plan-place nil))
  *sm*)

(defun main ()
  (pr2-init)
  (let* ((place (get-arg :place))
         (object (get-arg :object))
         (sm (plan-place object place)))
    (cond
      ((and sm (send sm :nodes))
       (exec-preemptive-task sm))
      (t
       (speak-jp "ごめんなさい。いまちょっと手が離せないです。")
       nil))))

(run-exit (main))
