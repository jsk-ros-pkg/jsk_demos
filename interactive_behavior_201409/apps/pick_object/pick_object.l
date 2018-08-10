#!/usr/bin/env roseus
;; pick_object.l

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")

(load "package://detect_cans_in_fridge_201202/euslisp/main.l")

(defun main ()
  (let ((object (get-arg :object)))
    (cond
      ((find object (list "wonda" "georgia" "boss" "iemon") :test #'string=)
       (send *ri* :clear-costmap)
       (demo :type object :use-arm :rarm))
      (t (speak-jp "ごめんなさい。いまちょっと手が離せないです。")))))

(run-exit (main))