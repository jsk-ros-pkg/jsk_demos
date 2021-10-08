#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")
(unless (boundp '*ri*)
  (spot-init))

(defvar *spot_autowalk_data_path*
      "/home/spot/spot_ws/src/jsk-ros-pkg/jsk_robot/jsk_spot_robot/spot_autowalk_data")

(defun autowalk-data-upload ()
  (setq *path*
        (ros::get-param "~path"
                        (format nil "~A/autowalk/eng2_73b2kitchen.walk" *spot_autowalk_data_path*)))
  (setq *init-waypoint* (floor (ros::get-param "~init_waypoint" 0)))
  (setq *upload* (ros::get-param "~upload" t))
  ;; Upload graphnav files to the robot.
  (if *upload*  (send *ri* :upload-path *path*))
  ;; Localize the robot in the map
  (ros::ros-info "initialize position with waypoint of ~A" *init-waypoint*)
  ;; (send *ri* :initial-localization-waypoint *init-waypoint*)
  (send *ri* :initial-localization-fiducial)
  )

(defun main ()
  (autowalk-data-upload)
  (send *ri* :navigate-to -1)
  (send *ri* :go-pos 0 0 90)
  (send *ri* :sit)
  (unix::sleep 1)
  (send *ri* :navigate-to 0)
  )
  
