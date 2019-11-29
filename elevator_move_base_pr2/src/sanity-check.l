(ros::roseus-add-msgs "sensor_msgs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")


(defun speak-subscription-error (topic)
  (ros::ros-error (format nil "Could not subscribe ~A." topic))
  (speak-jp
   (format nil "~A のトピックを subscribe できませんでした。" topic) :wait t))

(defun check-image-published (topic &optional (timeout 10))
  (when (one-shot-subscribe
         topic sensor_msgs::Image
         :timeout (* timeout 1000) :after-stamp (ros::time-now))
    (return-from check-image-published t))
  (speak-subscription-error topic)
  nil)

(defun check-point-cloud-published (topic &optional (timeout 10))
  (when (one-shot-subscribe
         topic sensor_msgs::PointCloud2
         :timeout (* timeout 1000) :after-stamp (ros::time-now))
    (return-from check-point-cloud-published t))
  (speak-subscription-error topic)
  nil)

(defun check-laser-scan-published (topic &optional (timeout 10))
  (when (one-shot-subscribe
         topic sensor_msgs::LaserScan
         :timeout (* timeout 1000) :after-stamp (ros::time-now))
    (return-from check-laser-scan-published t))
  (speak-subscription-error topic)
  nil)

(defun sanity-check (userdata)
  (let ((success t))
    (unless (boundp '*ri*)
      (pr2-init))
    (unless (check-image-published "/narrow_stereo/left/image_raw" 10)
      (setq success nil))
    (unless (check-image-published "/wide_stereo/left/image_raw" 10)
      (setq success nil))
    (unless
        (check-point-cloud-published "/kinect_head/depth_registered/points" 10)
      (setq success nil))
    (unless (check-laser-scan-published "/base_scan" 2)
      (setq success nil))
    (unless (check-laser-scan-published "/tilt_scan" 2)
      (setq success nil))
    success))
