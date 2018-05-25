#!/usr/bin/env roseus
;; pick_object.l

(ros::roseus "run_pick_object")

(setq *param* (ros::get-param "/action/parameters"))

(ros::ros-info "*param*: ~A" *param*)

(setq *object* (cdr (assoc "object" *param* :test #'string=)))

(load "package://detect_cans_in_fridge_201202/euslisp/main.l")

(defun main ()
  (cond
    ((find *object* (list "wanda" "georgia" "boss" "iemon") :test #'string=)
     (demo :type *object*))
    (t (speak-jp "ごめんなさい。いまちょっと手が離せないです。"))))


(if (main) (exit 0) (exit 1))
