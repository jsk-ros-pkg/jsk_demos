#!/usr/bin/env roseus

(ros::roseus "find_elevator_button")

(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :eng8-scene "package://jsk_maps/src/eng8-scene.l")

(load "package://elevator_move_base_pr2/src/posedetectiondb-client.l")


(defvar *tfl* (instance ros::transform-listener :init))
(unix::sleep 1) ;; wait for TF messages

(let ((frames (send *tfl* :get-frame-strings)))
  (cond ((find "eng2" frames :test #'string=)
         (setq *scene* (make-eng2-scene)))
        ((find "eng8" frames :test #'string=)
         (setq *scene* (make-eng8-scene)))
        (t
         (ros::ros-fatal
          "There is no building frame_id in TF tree, so exiting...")
         (exit))
        ))

(let ((objs (list (send *scene* :object "/elevator_inside_panel")
                  (send *scene* :object "/elevator_call_panel"))))
  (setq *clients* (init-posedetectiondb-client objs)))

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))
