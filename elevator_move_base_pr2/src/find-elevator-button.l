#!/usr/bin/env roseus

(load "package://elevator_move_base_pr2/src/posedetectiondb-client.l")
(load "package://elevator_move_base_pr2/src/utils.l")


(ros::roseus "find_elevator_button")

(defvar *scene* (make-scene-by-tf (instance ros::transform-listener :init)))
(defvar *clients* (init-posedetectiondb-client
                   (list (send *scene* :object "/elevator_inside_panel")
                         (send *scene* :object "/elevator_call_panel"))))

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))
