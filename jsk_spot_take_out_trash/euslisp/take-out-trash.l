#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")
(ros::roseus-add-msgs "spot_behavior_manager_msgs")

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
  (send *ri* :initial-localization-fiducial))

(defun go-to-garbagecollection-area ()
  (let ((behavior_msg nil))
    (ros::advertise "/spot_behavior_manager_demo/execute_behaviors/goal"
                    spot_behavior_manager_msgs::LeadPersonActionGoal)
    (setq behavior_msg (instance spot_behavior_manager_msgs::LeadPersonActionGoal :init))
    (send (send behavior_msg :goal) :target_node_id "eng2_GarbageCollectionArea_east")
    (ros::ros-info "Move to ~A!" (send (send behavior_msg :goal) :target_node_id))
    (send *ri* :speak-jp "ゴミ捨て場に向かいます")
    (ros::publish "/spot_behavior_manager_demo/execute_behaviors/goal" behavior_msg)))

(defun main ()
  ;; garbage colletion from trash cans
  (autowalk-data-upload)
  (send *ri* :navigate-to 1)
  (send *ri* :go-pos -1.4 0 -90)
  (send *ri* :sit)
  (unix::sleep 1)
  (send *ri* :navigate-to 0)
  (go-to-garbagecollection-area)
  )
