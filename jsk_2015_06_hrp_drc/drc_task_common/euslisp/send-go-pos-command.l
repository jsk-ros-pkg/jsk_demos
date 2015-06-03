#!/usr/bin/env roseus

(ros::roseus "send_go_pos_command")

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "drc_task_common")
(ros::roseus-add-srvs "drc_task_common")

(defun send-go-pos-command
  (dest-x dest-y dest-yaw) ;; [m] [m] [deg]
  (ros::ros-info "[send-go-pos-command] send go-pos command~%")
  (let* ((dest-coords
          (make-coords :pos (float-vector (* 1000 dest-x) (* 1000 dest-y) 0) :rpy (list (deg2rad dest-yaw) 0 0))))
    (warning-message 1 "Go pos to (~4,1f ~4,1f ~4,1f). OK?~%" dest-x dest-y dest-yaw)
    ;; service call for task state manager
    (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "ExecGoPos"))
    ;; send go pos command
    (ros::service-call "go_pos_command_from_rviz_interface"
                       (instance drc_task_common::GoPosCommandRequest :init :goal (ros::coords->tf-pose dest-coords)))
    ;; service call for task state manager
    (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "FinishGoPos"))
    )
  (ros::ros-info "[walk-to-object] finished send go-pos command~%")
  )

(while t
  (format t "input go-pos like (1 2 3) > ")
  (let ((sexp (read)))
    (if (and (listp sexp)
             (= (length sexp) 3)
             (numberp (car sexp))
             (numberp (cadr sexp))
             (numberp (caddr sexp))
             (< (abs (car sexp)) 3)
             (< (abs (cadr sexp)) 3))
        (if (y-or-n-p "go-pos ~A, OK?" sexp)
            (apply #'send-go-pos-command sexp))
      (warning-message 2 "malformed sexp~%"))))

