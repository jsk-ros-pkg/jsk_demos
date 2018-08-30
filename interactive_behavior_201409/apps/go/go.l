#!/usr/bin/env roseus
;; go.l

(ros::roseus "run_go")

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")

#|
(defun pr2-init (&optional (create-viewer))
  (setq *pr2* (pr2)
        *ri* (instance pr2-interface :init
                       :move-base-action-name "elevator_move_base"))
  (ros::spin-once)
  (send *ri* :spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))
|#

(defun plan-go (place)
  (setq *domain* (make-domain))
  (setq *problem* (make-problem :init '((ROBOT-AT START))
                                :goal `((ROBOT-AT ,place))))
  (unless (setq *sm* (plan-task *domain* *problem* :debug t))
    (send *ri* :speak-jp "計画に失敗しました")
    (return-from plan-go nil))
  *sm*)

(defun main ()
  (pr2-init)
  (let* ((place (get-arg :place))
         (sm (plan-go place)))
    (cond
      (sm
       (exec-preemptive-task sm))
      (t
       (speak-jp "ごめんなさい。いまちょっと手が離せないです。")
       nil))))

(run-exit (main))
