#!/usr/bin/env roseus

(ros::roseus-add-srvs "elevator_operation")
(ros::roseus-add-msgs "move_base_msgs")

(load "package://elevator_operation/euslisp/utils.l")


(defun handler-start-look-at-target (req)
  (let (success
        (res (instance elevator_operation::LookAtTargetResponse :init)))
    (setq success (look-at-target-frame (send req :frame_id) :robot-type *robot-type*))
    (send res :success success)
    res))

(defun main()
  (ros::roseus "look_at_elevator_server")
  (setq *robot-type* (ros::get-param "~robot_type" ""))
  (cond
    ((equal *robot-type* "fetch")
      (load "package://fetcheus/fetch-interface.l")
      (load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
      (fetch-init)
      (setq *robot* *fetch*))
    (t
      (setq *move-base-action-client* (instance ros::simple-action-client :init "/move_base" move_base_msgs::MoveBaseAction))
      (send *move-base-action-client* :wait-for-server)
      )
    )
  (setq *tfl* (instance ros::transform-listener :init))
  (ros::advertise-service "~look_at_target" elevator_operation::LookAtTarget #'handler-start-look-at-target)
  (ros::ros-info "initialized")
  (ros::spin)
  )

(main)
