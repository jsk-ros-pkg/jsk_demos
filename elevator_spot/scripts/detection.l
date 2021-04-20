#!/usr/bin/env roseus
(load "package://spoteus/spot-interface.l")

(unless (boundp '*ri*) (setq *ri* (instance spot-interface :init)))

(setq *detection* t)

(defun check-elevator-open (camera &key (timeout 300))
  ;; This function is cited from jsk_demos/elevator_move_base_pr2
  ;; NEED TO CHANGE TOPIC NAME
  (let ((front-topic
         "/check_elevator_open/front_octree_change_publisher/octree_change_result")
        (back-topic
         "/check_elevator_open/back_octree_change_publisher/octree_change_result")
        (change-thre (ros::get-param "change_threshold" 250))
        (change-pc nil) time-start topic)
    ;; change topic-name by camera input
    (cond ((string= camera "front")
           (setq topic front-topic))
          ((string= camera "back")
           (setq topic back-topic))
          (t (ros::ros-error "The robot cannot detect the elevator opening by the camera.")))
    (setq time-start (ros::time-now))
    (ros::subscribe
     topic sensor_msgs::PointCloud2
     #'(lambda (msg)
         (when msg
           (let ((stamp (send msg :header :stamp)))
             ;; Wait until point cloud becomes stable
             (when (ros::time> stamp (ros::time+ time-start (ros::time 0.5)))
               (setq change-pc (* (send msg :height) (send msg :width)))
               (ros::ros-info "pointcloud change: ~A, open/closed threshold: ~A"
                              change-pc change-thre))))))
    (ros::rate 10)
    (while (or (null change-pc) (< change-pc change-thre))
      (ros::spin-once)
      (when (ros::time> (ros::time-now)
                        (ros::time+ time-start (ros::time timeout)))
        (ros::warn
         (format nil "Could not detect door opening in ~A [sec]." timeout))
        (send *ri* :speak-jp "ドアが開いていません。")
        (ros::unsubscribe topic)
        (return-from check-elevator-open nil))
      (ros::sleep))
    (ros::unsubscribe topic)
    (ros::ros-info "Door opened.")
    (send *ri* :speak-jp "ドアが開きました。")
    (clear-costmap)
    t))

(defun check-current-floor()
  ;; Detect the current floor by using IMU
  )
