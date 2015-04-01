#!/usr/bin/env roseus

(ros::roseus "eus_commandserver")

(load "package://drc_task_common/euslisp/robot-util.l")
(generate-robot "HRP2JSKNT" :generate-ri? t)

(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "drc_task_common")
(ros::roseus-add-srvs "drc_task_common")

(defun eus-command-init
  ()
  (load "package://drc_task_common/euslisp/set-recog-param.l")
  (ros::advertise-service "/eus_command" drc_task_common::EusCommand #'eus-command-call)

  (warning-message 2 "[eus-command-server] initialize eus-command-server.~%")
  )

(defun eus-command-call
  (req)
  (let* ((res (send req :response))
         eus-res)
    (warning-message 2 "[eus-command-server] eus command: ~a~%" (send req :command))
    (setq eus-res (eval (read-from-string (send req :command))))
    (warning-message 2 "[eus-command-server] eus return: ~a~%" eus-res)
    res))


(eus-command-init)
(ros::rate 10)
(while t
  (ros::sleep)
  (ros::spin-once)
  )
