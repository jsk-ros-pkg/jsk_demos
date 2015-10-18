;;#!/usr/bin/env roseus

(ros::roseus "checkerboard_subscriber")
(ros::roseus-add-msgs "std_msgs")

(setq *target-coords* (make-coords)) ;;prepare checkerbord coords for callbadck
(defun get-camera-coords (&optional (stamp (ros::time 0)))
  (if (send *tfl* :wait-for-transform "/BODY" "/left_camera_optical_frame" stamp 5)
      (send (send (send *robot* :link "BODY") :copy-worldcoords) :transform
            (send *tfl* :lookup-transform "/BODY" "/left_camera_optical_frame" stamp))
    (progn
      (ros::ros-warn "Failed to lookup tf with exact timestamp")
      (send (send (send *robot* :link "BODY") :copy-worldcoords) :transform
            (send *tfl* :lookup-transform "/BODY" "/left_camera_optical_frame" (ros::time 0))))))


(defun get-target-coords-callback (msg)
  (let (target-coords)
    (setq *target-coords* 
          (send (get-camera-coords (send msg :header :stamp)) :transform (ros::tf-pose->coords (send msg :pose))))
    target-coords
    ))
(defun get-target-coords ()
  (ros::spin-once)
  (ros::subscribe "door_handle" geometry_msgs::PoseStamped #'get-target-coords-callback)
  *target-coords*
  )
