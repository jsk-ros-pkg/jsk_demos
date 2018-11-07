#!/usr/bin/env roseus


(defun change-floor (target-floor
                     &key
                     (topicname "/initialpose3d")
                     (base-frame-name "base_footprint")
                     (map-tf-mux "/map_tf_mux/selected"))
  "change floor by publishing geometry_msgs::PoseWithCovarianceStamped to topicname.
   example of target-floor's name is /eng2/2f, /eng2/7f.

   (change-floor \"/eng2/2f\")"
  (ros::advertise topicname geometry_msgs::PoseWithCovarianceStamped 1)
  (unix:usleep (* 1000 500))
  (let ((current-floor-frame ;; for example /eng2/7f_tf
         (one-shot-subscribe map-tf-mux std_msgs::String))
        (robot-pose (send *tfl* :lookup-transform current-floor-frame base-frame-name (ros::time 0))))
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
      (ros::publish topicname msg))))
