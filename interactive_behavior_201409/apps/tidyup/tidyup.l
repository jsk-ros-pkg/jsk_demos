#!/usr/bin/env roseus
;; tidyup.l

(ros::roseus "run_tidyup")

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")
(require :speak "package://pr2eus/speak.l")
(load "package://jsk_2017_12_pr2_coe/euslisp/setup.l")

(defun kitchen-demo ()
  (exec-state-machine (make-state-machine)))

(defun plan-tidyup (object place)
  (setq *domain* (make-domain))
  (setq *problem* (make-problem :init '((= (total-cost) 0)
                                        (AT START)
                                        (ON CUP TABLE)
                                        )
                                :goal `((ON ,object ,place))))
  (unless (setq *sm* (plan-task *domain* *problem* :debug t))
    (ros::ros-error "failed to plan")
    (return-from plan-tidyup nil))
  *sm*)


(defun main ()
  (let* ((place (get-arg :place))
         (object (get-arg :object))
         (sm (plan-tidyup object place)))

    (cond
      (sm (exec-preemptive-task sm))
      (t (speak-jp "ごめんなさい。いまちょっと手が離せないです。")))
    ))

(run-exit (main))
