#!/usr/bin/env roseus
;; deliver.l

(ros::roseus "deliver")

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")

(defun plan-deliver (object person)
  (let ((place (person->place person)))
    (setq *domain* (make-domain))
    (setq *problem* (make-problem :init '((ROBOT-AT START))
                                  :goal `((ON ,object ,place))
                                  :items `(,object)))
  (unless (setq *sm* (plan-task *domain* *problem* :debug t))
    (ros::ros-error "failed to plan")
    (return-from plan-deliver nil))
  *sm*))


(defun main ()
  (let* ((object (get-arg :object))
         (person (get-arg :person))
         (sm (plan-deliver object person)))
    (cond
      (sm (exec-preemptive-task sm))
      (t (speak-jp "ごめんなさい。いまちょっと手が離せないです。")))
    ))

(run-exit (main))
