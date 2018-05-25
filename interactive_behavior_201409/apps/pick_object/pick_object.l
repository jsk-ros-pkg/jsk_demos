#!/usr/bin/env roseus
;; pick_object.l

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")

(load "package://detect_cans_in_fridge_201202/euslisp/main.l")

(defun main ()
  (let ((object (cdr (assoc "object" *param* :test #'string=))))
    (cond
      ((find *object* (list "wanda" "georgia" "boss" "iemon") :test #'string=)
       (demo :type *object*))
      (t (speak-jp "ごめんなさい。いまちょっと手が離せないです。")))))

(if (main) (exit 0) (exit 1))
