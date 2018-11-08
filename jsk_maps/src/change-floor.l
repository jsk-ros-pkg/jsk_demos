(defun change-costmap-publish-frequency (&optional (frequency 0.0)
                                                   &key
                                                   (node-name "move_base_node")
                                                   (costmap-name "global_costmap"))
  "change costmap's publish frequency"
  (when (< frequency 0.0)
    (ros::ros-error (format nil "frequency should be greater than or equal to 0.0"))
    (return-from change-costmap-publish-frequency nil))
  (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
    (send req :config :doubles
          (list (instance dynamic_reconfigure::DoubleParameter :init
                          :name "publish_frequency" :value frequency)))
    (ros::service-call (format nil "~A/~A/set_parameters" node-name costmap-name) req))
  t)


(defun change-floor (target-floor
                     &key
                     (floor-frame "/map")
                     (topicname "/inamenitialpose3d")
                     (base-frame-name "base_footprint")
                     (map-tf-mux "/map_tf_mux/selected"))
  "change floor by publishing geometry_msgs::PoseWithCovarianceStamped to topicname.
   example of target-floor's name is /eng2/2f, /eng2/7f.

   (change-floor \"/eng2/2f\")"
  (ros::advertise topicname geometry_msgs::PoseWithCovarianceStamped 1)
  (unix:usleep (* 1000 500))
  (let (robot-pose)
    (when (not (boundp '*tfl*))
      (ros::ros-error (format nil "transform listener(*tfl*) is not initialized."))
      (return-from change-floor nil))
    (setq robot-pose
          (send *tfl* :lookup-transform floor-frame base-frame-name (ros::time 0)))
    (when (null robot-pose)
      (return-from change-floor nil))
    (let* ((msg (instance geometry_msgs::PoseWithCovarianceStamped :init))
           (covariance (send msg :pose :covariance)))
      (send msg :header :frame_id target-floor)
      (send msg :header :stamp (ros::time-now))
      (setf (elt covariance 0) 0.05)
      (setf (elt covariance 7) 0.05)
      (setf (elt covariance 21) 0.02)
      (send msg :pose :pose (ros::coords->tf-pose robot-pose))
      (ros::publish topicname msg))
    t))
