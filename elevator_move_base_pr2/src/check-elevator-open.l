(ros::roseus-add-msgs "sensor_msgs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")


(defun check-elevator-open (&key (timeout 300))
  (let ((topic
         "/check_elevator_open/octree_change_publisher/octree_change_result")
        (change-thre (ros::get-param "change_threshold" 700))
        (change-pc nil) time-start)
    (unless (and (boundp '*pr2*) (boundp '*ri*))
      (ros::ros-error "*pr2* or *ri* is not bound.")
      (return-from check-elevator-open nil))
    (send *pr2* :head :angle-vector #f(0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 400)
    (send *ri* :wait-interpolation)

    (ros::subscribe topic sensor_msgs::PointCloud2
     #'(lambda (msg) (setq change-pc (* (send msg :height) (send msg :width)))))
    (unix:usleep (* 300 1000))  ;; Wait until point cloud becomes stable

    (setq time-start (ros::time-now))
    (ros::rate 10)
    (while (or
            (null change-pc)
            (< change-pc change-thre))
      (when (ros::time> (ros::time-now)
                        (ros::time+ time-start (ros::time timeout)))
        (ros::warn
         (format nil "Could not detect door opening in ~A [sec]." timeout))
        (speak-jp "ドアが開いていません。")
        (return-from check-elevator-open nil))
      (ros::ros-info "point cloud change: ~A, threshold of open/closed: ~A"
                     change-pc change-thre)
      (ros::spin-once)
      (ros::sleep))

    (ros::unsubscribe topic)
    (ros::ros-info "Door opened.")
    (speak-jp "ドアが開きました。")
    (clear-costmap)
    (unix:usleep (* 500 1000)) ;; wait until door open
    t))
