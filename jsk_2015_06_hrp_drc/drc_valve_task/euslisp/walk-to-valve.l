#!/usr/bin/env roseus

(load "package://drc_valve_task/euslisp/valve-task-util.l")

(ros::roseus "variable_model_publisher")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "std_msgs")

(defun init
  ()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
  (hrp2jsknts-init))

(defun main
  ()
  (defvar *tfl* (instance ros::transform-listener :init))
  (ros::subscribe "/go_pos_command" std_msgs::bool #'go-pos-command-cb)
  (warning-message 2 "====================================~%")
  (warning-message 2 "Ready to go pos.~%")
  (send *ri* :set-gait-generator-param :stride-parameter (float-vector 0.10 0.05 10.0))
  (while t
    (ros::spin-once))
  )

(defun go-pos-command-cb
  (msg)
  (let* (dest-coords
         dest-x dest-y dest-yaw)
    (setq dest-coords
          (send (send *tfl* :lookup-transform "hrp2_marker_root" "ground" (instance ros::time :init)) :transformation (make-coords) :local))
    (setq dest-x (* 0.001 (elt (send dest-coords :pos) 0)))
    (setq dest-y (* 0.001 (elt (send dest-coords :pos) 1)))
    (setq dest-yaw (rad2deg (elt (car (rpy-angle (send dest-coords :rot))) 0)))
    (when (send msg :data)
      (warning-message 2 "Go pos to (~a ~a ~a). Send to real robot?~%" dest-x dest-y dest-yaw)
      (when (y-or-n-p-org)
        (send *ri* :go-pos dest-x dest-y dest-yaw))
      ))
      (warning-message 2 "Finished~%")
      (warning-message 2 "====================================~%")
      (warning-message 2 "Ready to go pos.~%")
  )


(init)
(main)
