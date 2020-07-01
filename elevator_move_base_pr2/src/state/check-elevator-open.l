(ros::roseus-add-msgs "sensor_msgs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")


;; (defvar *pr2*)
;; (defvar *ri*)


(defun check-elevator-open (&key (timeout 300))
  (let ((topic
         "/check_elevator_open/octree_change_publisher/octree_change_result")
        (change-thre (ros::get-param "change_threshold" 400))
        (change-pc nil) time-start)
    (send *pr2* :head :angle-vector #f(0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 400)
    (send *ri* :wait-interpolation)

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
        (speak-jp "ドアが開いていません。")
        (ros::unsubscribe topic)
        (return-from check-elevator-open nil))
      (ros::sleep))

    (ros::unsubscribe topic)
    (ros::ros-info "Door opened.")
    (speak-jp "ドアが開きました。")
    (clear-costmap)
    t))
