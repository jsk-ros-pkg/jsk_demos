#!/usr/bin/env roseus

(ros::roseus "checkerboard_subscriber")
(ros::roseus-add-msgs "std_msgs")

(setq *target-coords* nil) ;;prepare checkerbord coords for callbadck
(defun get-camera-coords (&optional (stamp (ros::time 0)))
  (if (send *tfl* :wait-for-transform "/BODY" "/left_camera_optical_frame" stamp 10)
      (send (send (send *robot* :link "BODY") :copy-worldcoords) :transform
            (send *tfl* :lookup-transform "/BODY" "/left_camera_optical_frame" stamp))
    (progn
      (ros::ros-warn "Failed to lookup tf with exact timestamp")
      (send (send (send *robot* :link "BODY") :copy-worldcoords) :transform
            (send *tfl* :lookup-transform "/BODY" "/left_camera_optical_frame" (ros::time 0))))))
(defun get-target-coords-callback (msg)
  (let ()
    (setq *target-coords* 
          (send (get-camera-coords (send msg :header :stamp)) :transform (ros::tf-pose->coords (send msg :pose))))
    ))
(defun get-target-coords (&optional (timeout 2))
  (let ()
  (ros::spin-once)
  (setq *target-coords* nil)
  (setq start-time (ros::time-now))
  (while (ros::ok)
    (let ((current-time (ros::time-now)))
      (if *target-coords*
          (return-from get-target-coords *target-coords*))
      (if (> (send (ros::time- current-time start-time) :to-sec) timeout)
          (return-from get-target-coords))
      (ros::sleep)
      (ros::spin-once))
    )))
(ros::subscribe "door_handle" geometry_msgs::PoseStamped #'get-target-coords-callback)
(ros::rate 10)



 
