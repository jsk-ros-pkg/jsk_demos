#!/usr/bin/env roseus

(ros::roseus "walk_to_object")
(load "package://drc_task_common/euslisp/robot-util.l")

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_rviz_plugins")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "drc_task_common")
(ros::roseus-add-srvs "drc_task_common")

(defun walk-to-object-init
  ()
  ;; select ocs/fc
  (setq *use-robot-interface-flag* (ros::get-param "~use_robot_interface" nil))
  (setq *robot-name* (string-upcase (unix::getenv "ROBOT")))
  (if *use-robot-interface-flag*
      (generate-robot *robot-name* :generate-ri? *use-robot-interface-flag*)
      )
  (setq *tfl* (instance ros::transform-listener :init))
  (setq *go-pos-command-flag* nil)
  (ros::subscribe "/go_pos_command" std_msgs::empty #'go-pos-command)
  (warning-message 2 "[walk-to-object] initialize walk-to-object.~%")
  )

(defun walk-to-object-one
  ()
  (warning-message 2 "[walk-to-object] send go-pos command~%")
  (let* (dest-coords
         dest-x dest-y dest-yaw)
    (while (not dest-coords)
      (let* ((ts (ros::time-now)))
        (send *tfl* :wait-for-transform "robot_marker_root" "ground" ts 1)
        (setq dest-coords
              (send *tfl* :lookup-transform "robot_marker_root" "ground" ts)))
      (ros::sleep)
      (ros::spin-once))
    (setq dest-coords
          (send dest-coords :transformation (make-coords) :local))
    (setq dest-x (* 0.001 (elt (send dest-coords :pos) 0)))
    (setq dest-y (* 0.001 (elt (send dest-coords :pos) 1)))
    (setq dest-yaw (rad2deg (elt (car (rpy-angle (send dest-coords :rot))) 0)))
    (let* ((title
            (format nil "Go pos to (~4,3f ~4,3f ~4,3f). OK?~%" dest-x dest-y dest-yaw))
           (menu-list (list "cancel" "yes"))
           (yes-index 1)
           (req (instance drc_task_common::RvizMenuCallRequest :init :title title :menu_list menu-list))
           (res (ros::service-call "rviz_menu_call" req))
           (res-index (send res :index)))
      (warning-message 2 title)
      (when (equal res-index yes-index)
        ;; service call for task state manager
        (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "ExecGoPos"))
        ;; send go pos command
        (cond
         ;; when robot-interface is available (FC)
         (*use-robot-interface-flag*
          (send *ri* :go-pos dest-x dest-y dest-yaw))
         ;; when robot-interface is not available (OCS)
         (t
          (ros::service-call "go_pos_command_from_rviz_interface"
                             (instance drc_task_common::GoPosCommandRequest :init :goal (ros::coords->tf-pose dest-coords)))))
        ;; service call for task state manager
        (ros::service-call "call_task_state_event" (instance drc_task_common::StringRequestRequest :init :data "FinishGoPos"))
        ))
    )
  (setq *go-pos-command-flag* nil)
  (warning-message 2 "[walk-to-object] finished sending go-pos command~%")
  )

(defun go-pos-command
  (msg)
  (setq *go-pos-command-flag* t))


(walk-to-object-init)
(ros::rate 10)
(while t
  (ros::sleep)
  (ros::spin-once)
  (when *go-pos-command-flag*
    (walk-to-object-one))
  )
