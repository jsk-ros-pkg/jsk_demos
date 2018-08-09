#!/usr/bin/env roseus
;; place.l

(ros::roseus "run_place")

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")

(defun plan-place (object place)
  (setq *domain* (make-domain))
  (setq *problem* (make-problem :init '((= (total-cost) 0)
                                        (AT START)
                                        (ON CUP TABLE)
                                        )
                                :goal `((ON ,object ,place))))
  (unless (setq *sm* (plan-task *domain* *problem* :debug t))
    (ros::ros-error "failed to plan")
    (return-from plan-place nil))
  *sm*)

(defun main ()
  (let* ((place (get-arg :place))
         (object (get-arg :object))
         (sm (plan-place object place)))
    (pr2-init)
    (cond
      (sm
       (exec-preemptive-task sm))
      (t
       (speak-jp "ごめんなさい。いまちょっと手が離せないです。")
       nil))))

(run-main (main))
