#!/usr/bin/env roseus

(ros::roseus "find_elevator_button")

(load "package://elevator_move_base_pr2/src/posedetectiondb-client.l")
(load "package://elevator_move_base_pr2/src/elevator-buttons.l")

(load "package://pr2eus/pr2-interface.l")

(defvar *tfl* (instance ros::transform-listener :init))
(unix::sleep 1) ;; wait for TF messages

(let ((frames (send *tfl* :get-frame-strings)))
  (cond ((find "eng2" frames :test #'string=)
         (setq *scene* (make-eng2-scene)))
        ((find "eng8" frames :test #'string=)
         (setq *scene* (make-eng8-scene)))
        (t
         (ros::ros-error "There is no building frame-id in TF tree exiting...")
         (exit))
        ))

(let ((objs (list (send *scene* :object "/elevator_inside_panel")
                  (send *scene* :object "/elevator_call_panel"))))
  (setq *clients* (init-posedetectiondb-client objs)))


(ros::rate 10)
(while (ros::ok)
 (ros::spin-once)
 (ros::sleep))
