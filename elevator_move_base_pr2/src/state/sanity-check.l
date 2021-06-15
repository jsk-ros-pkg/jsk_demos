(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "sensor_msgs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(load "package://elevator_move_base_pr2/src/utils.l")


;; (defvar *ri*)


(defun speak-subscription-error (topic)
  (ros::ros-error (format nil "Could not subscribe ~A." topic))
  ;; e.g. "/narrow_stereo/left/image_rect" -> "narrow stereo left image rect"
  (speak-jp
   (format nil "~Aのトピックをsubscribeできませんでした。"
           (string-trim
            " " (substitute #\Space #\_ (substitute #\Space #\/ topic))))
   :wait t))

(defun check-topic-published (topic type &optional (timeout 10))
  (ros::ros-info (format nil "Start subscribing ~A..." topic))
  (when (one-shot-subscribe
         topic type :timeout (* timeout 1000) :after-stamp (ros::time-now))
    (return-from check-topic-published t))
  (speak-subscription-error topic)
  nil)

(defun sanity-check (userdata)
  (let ((success t))
    (ros::ros-info "Start sanity-check...")
    (unless (boundp '*ri*)
      (pr2-init))

    ;; Sensors
    (unless (check-topic-published
             "/narrow_stereo/left/image_rect" sensor_msgs::Image 10)
      (setq success nil))
    (unless (check-topic-published
             "/wide_stereo/left/image_rect_color" sensor_msgs::Image 10)
      (setq success nil))
    (unless (check-topic-published
             "/kinect_head/depth_registered/points" sensor_msgs::PointCloud2 10)
      (setq success nil))
    (unless (check-topic-published "/base_scan" sensor_msgs::LaserScan 2)
      (setq success nil))
    (unless (check-topic-published "/tilt_scan" sensor_msgs::LaserScan 2)
      (setq success nil))

    ;; For look-at-human
    (unless (check-topic-published "edgetpu_human_pose_estimator/output/poses"
                                   jsk_recognition_msgs::PeoplePoseArray 5)
      (setq success nil))

    ;; For door button recognition with FCN
    (call-service "door_button/pass_through_image/request"
                  (instance std_srvs::EmptyRequest :init))
    (unless (check-topic-published
             "door_button/fcn_object_segmentation/output" sensor_msgs::Image 20)
      (setq success nil))
    (call-service "door_button/pass_through_image/stop"
                  (instance std_srvs::EmptyRequest :init))

    ;; For human in mirror recognition with FCN
    (call-service "human_in_mirror/pass_through_kinect_rgb/request"
                  (instance std_srvs::EmptyRequest :init))
    (call-service "human_in_mirror/pass_through_kinect_depth/request"
                  (instance std_srvs::EmptyRequest :init))
    (unless (check-topic-published
             "human_in_mirror/find_human_in_mirror/output/inside_mirror"
             jsk_recognition_msgs::BoolStamped 20)
      (setq success nil))
    (unless (check-topic-published
             "human_in_mirror/find_human_in_mirror/output/outside_mirror"
             jsk_recognition_msgs::BoolStamped 20)
      (setq success nil))
    (call-service "human_in_mirror/pass_through_kinect_rgb/stop"
                  (instance std_srvs::EmptyRequest :init))
    (call-service "human_in_mirror/pass_through_kinect_depth/stop"
                  (instance std_srvs::EmptyRequest :init))

    (if success
        (ros::ros-info "Successfully finished sanity-check.")
      (ros::ros-error "Failed to execute sanity-check."))
    success))
