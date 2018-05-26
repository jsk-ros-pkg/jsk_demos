#!/usr/bin/env roseus
;; tidyup.l

(ros::roseus "run_tidyup")

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")
(require :speak "package://pr2eus/speak.l")
(load "package://jsk_2017_12_pr2_coe/euslisp/setup.l")

(defun kitchen-demo ()
  (exec-state-machine (make-state-machine)))


(defun main ()
  (let ((place (get-arg :place)))
    (cond
      ((find place (list "kitchen") :test #'string=)
       (kitchen-demo))
      (t (speak-jp "ごめんなさい。いまちょっと手が離せないです。")))))

(if (main) (exit 0) (exit 1))
