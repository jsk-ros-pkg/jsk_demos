#!/usr/bin/env roseus

(ros::roseus "eus_commandserver")

(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")
(ros::roseus-add-msgs "jsk_pcl_ros")
(ros::roseus-add-srvs "jsk_pcl_ros")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "drc_task_common")
(ros::roseus-add-srvs "drc_task_common")

(defun eus-command-init
  ()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
  (load "package://jsk_hrp2_ros_bridge/scripts/thk-hand-utils.l")
  (load "package://drc_task_common/euslisp/set-recog-param.l")
  (hrp2jsknts-init)
  (setq *robot* *hrp2jsknts*)
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
(while t
  (ros::spin-once)
  )
