#!/usr/bin/env roseus
;; pick_object.l

(require :app-utils "package://jsk_demo_common/euslisp/app-utils.l")
(require :speak "package://pr2eus/speak.l")

(defun main ()
  (let ((object (string-downcase (get-arg :object))))
    ;; set default object: georgia
    (if (null object) (setq object "georgia"))
    (cond
      ((find object (list "wonda" "georgia" "boss" "iemon") :test #'string=)
       (require :fridge-demo-smach-main
                "package://detect_cans_in_fridge_201202/euslisp/fridge-demo-smach-main.l")
       (send *ri* :clear-costmap)
       (fridge-demo-smach-main :can-type object
                               :use-arm :larm
                               :use-base nil))
      (t (speak-jp "ごめんなさい。いまちょっと手が離せないです。")
       nil))
))

(ros::roseus "pick_object")
(run-exit (main))
