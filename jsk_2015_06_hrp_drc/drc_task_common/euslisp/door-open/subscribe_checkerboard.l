;;#!/usr/bin/env roseus
;; listener.l
;; Author: 

(ros::roseus "checkerboard_subscriber")
(ros::roseus-add-msgs "std_msgs")

(setq *checkerboard-coords* (make-coords)) ;;prepare checkerbord coords for callbadck
(defun get-camera-coords()
  (send (send (send *robot* :link "BODY") :copy-worldcoords) :transform
        (send *tfl* :lookup-transform "/BODY" "/HEAD_LEFT_CAMERA"
              (ros::time 0)
              )))
(defun get-checkerboard-coords-callback (msg)
  (setq *checkerboard-coords* (send (get-camera-coords) :transform (ros::tf-pose->coords (send msg :pose))))
  )
(defun get-checkerboard-coords ()
  (ros::spin-once)
  (ros::subscribe "checkerdetector/objectdetection_pose" geometry_msgs::PoseStamped #'get-checkerboard-coords-callback)
  (print (send *checkerboard-coords* :pos))
  )
(defun get-target-coords (&key (file-path "./config_files/target_coords_to_checkerboard.l"))
  "input relative coords to checkerboard and get world-coords of target object"
  (let ();;(relative-coords-to-checkerboard)
    (load file-path)
    (get-checkerboard-coords)
    ;;(setq relative-coords-to-checkerboard (make-coords :pos (send relative-coords-to-checkerboard :pos) :rot (send relative-coords-to-checkerboard :rot)))
    (setq object-world-coords (send *checkerboard-coords* :transformation *relative-coords-to-checkerboard*))
    ))

(defun set-relative-coords-to-checkerboard (target-coords &key (file-path "./config_files/target_coords_to_checkerboard.l"))
  (let ();;(relative-coords-to-checkerboard)
   (get-checkerboard-coords)
   (setq *relative-coords-to-checkerboard* (send (send (send *checkerboard-coords* :inverse-transformation) :transform target-coords) :copy-worldcoords))
   (dump-loadable-structure file-path *relative-coords-to-checkerboard*)
   ))
