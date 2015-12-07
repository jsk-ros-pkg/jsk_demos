#!/usr/bin/env roseus

(ros::roseus "conservative_door_detection_sample")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "geometry_msgs")
(setq *door-handle-msg* nil)
(defun run-detector (&optional (timeout 20))
  (let ((req (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "/tilt_laser_listener/clear_cache" req))
  (let ((req (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "/laser_snapshot/request" req))
  (ros::spin-once)                      ;dummy callback
  (setq *door-handle-msg* nil)
  (let ((start-time (ros::time-now)))
    (while (ros::ok)
      (let ((current-time (ros::time-now)))
        (if (> (send (ros::time- current-time start-time) :to-sec) timeout)
            (return-from run-detector nil)))
      (if *door-handle-msg*
          (return-from run-detector *door-handle-msg*))
      (ros::sleep)
      (ros::spin-once))
    ))

(defun door-handle-callback (msg)
  (setq *door-handle-msg* msg))
(ros::subscribe "/door_handle" geometry_msgs::PoseStamped #'door-handle-callback)
(ros::rate 5)


