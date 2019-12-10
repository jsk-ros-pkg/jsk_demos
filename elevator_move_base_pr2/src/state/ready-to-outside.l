(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "roseus")

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(load "package://elevator_move_base_pr2/src/ros-callback-manager.l")
(load "package://elevator_move_base_pr2/src/state/check-elevator-open.l")
(load "package://elevator_move_base_pr2/src/state/push-elevator-button.l")
(load "package://elevator_move_base_pr2/src/utils.l")


(defvar *floors*)
(defvar *panel*)
;; (defvar *pr2*)
;; (defvar *ri*)
(defvar *scene*)
(defvar *tfl*)


(defun check-target-floor-arrived (target-floor &key (timeout 10))
  (let ((topic
         (instance ros-callback-message
                   :init roseus::StringStamped "/elevator_number/result"))
        (prev-reset-time (ros::time-now))
        (prev-floor nil)
        (cur-floor nil)
        button-coords
        (button-state (instance light-button-cb-class :init)))
    (call-service
     "/narrow_stereo/left/pass_through_feature_to_inside_panel/request"
     (instance std_srvs::EmptyRequest :init))
    (call-service "/wide_stereo/left/pass_through_image_rect_color/request"
                  (instance std_srvs::EmptyRequest :init))
    (ros::rate 10)
    (while (ros::ok)
      (update 1)
      (setq button-coords
            (find-if #'(lambda (o) (string= target-floor (send o :name)))
                     (flatten (send *panel* :inheritance))))
      (set-view-target button-coords (send *pr2* :wide_stereo-left)
                       "wide_stereo_optical_frame")
      (ros::spin-once)
      (when (send topic :msg)
        (setq cur-floor (string-upcase (send topic :msg :data))))
      (unless (equal cur-floor prev-floor)  ;; Elevator moved, so reset time.
        (ros::ros-info (format nil "Current floor: ~A" cur-floor))
        (setq prev-reset-time (ros::time-now))
        (setq prev-floor cur-floor))
      (when (and (ros::time> (ros::time-now)
                             (ros::time+ prev-reset-time (ros::time timeout)))
                 (< (send button-state :value) 0.5))
        (ros::warn
         (format nil "Elevator has not moved in recent ~A [sec]." timeout))
        (speak-jp "エレベータが動いていません。")
        (call-service
         "/narrow_stereo/left/pass_through_feature_to_inside_panel/stop"
         (instance std_srvs::EmptyRequest :init))
        (call-service "/narrow_stereo/left/pass_through_image_rect/stop"
                      (instance std_srvs::EmptyRequest :init))
        (call-service "/wide_stereo/left/pass_through_image_rect_color/stop"
                      (instance std_srvs::EmptyRequest :init))
        (return-from check-target-floor-arrived nil))
      (when (and cur-floor
                 (substringp (concatenate string "/" target-floor) cur-floor))
        (call-service
         "/narrow_stereo/left/pass_through_feature_to_inside_panel/stop"
         (instance std_srvs::EmptyRequest :init))
        (call-service "/narrow_stereo/left/pass_through_image_rect/stop"
                      (instance std_srvs::EmptyRequest :init))
        (call-service "/wide_stereo/left/pass_through_image_rect_color/stop"
                      (instance std_srvs::EmptyRequest :init))
        (return-from check-target-floor-arrived t))
      (ros::sleep)
      )))

(defun change-floor (target-floor scene tfl
                     &optional (topicname "/initialpose3d"))
  (ros::advertise topicname geometry_msgs::PoseWithCovarianceStamped 1)
  (unix:usleep (* 100 1000))  ;; Wait for publisher to be ready
  (let* ((msg (instance geometry_msgs::PoseWithCovarianceStamped :init))
         (current-floor-frame (check-current-floor scene tfl))
         (robot-pose (send tfl :lookup-transform
                           current-floor-frame "base_footprint" (ros::time 0))))
    (unless robot-pose
      (return-from change-floor nil))
    (send msg :header :frame_id target-floor)
    (send msg :header :stamp (ros::time-now))
    (let ((cov (send msg :pose :covariance)))
      (setf (elt cov 0) 0.05)
      (setf (elt cov 7) 0.05)
      (setf (elt cov 21) 0.02))
    (send msg :pose :pose (ros::coords->tf-pose robot-pose))
    (ros::publish topicname msg)))

(defun ready-to-outside (userdata)
  (call-service
   "/narrow_stereo/left/pass_through_feature_to_inside_panel/request"
   (instance std_srvs::EmptyRequest :init))
  (call-service "/narrow_stereo/left/pass_through_image_rect/request"
                (instance std_srvs::EmptyRequest :init))
  (clear-costmap)
  (tolerance-loose)
  (send *ri* :go-pos-unsafe 0 0 90)  ;; Unsafe for faster turning left

  (update-robot-position)
  ;; Look at panel
  (let ((look-target
         (or (find-if #'(lambda (x) (string= (send x :name) "/elevator_number"))
                      (send *panel* :descendants))
             *panel*)))
    (send *pr2* :head :look-at (send look-target :worldpos))
    ;; Move fast but converge to the target angle vector
    (while (not (eps-v= (send *pr2* :angle-vector)
                        (send *ri* :state :potentio-vector)
                        1.0))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
      (send *ri* :wait-interpolation)))

  (let* ((target-floor (cdr (assoc 'target-floor userdata))))
    (unless (check-target-floor-arrived target-floor :timeout 10)
      (send *ri* :go-pos-unsafe 0 0 -90)  ;; Unsafe for faster turning right
      (return-from ready-to-outside :fail))

    (dolist (floor *floors*)
      (when (equal target-floor (elt floor 0))
        (ros::ros-info (format nil "Arrived at ~A." (elt floor 0)))
        (speak-jp (concatenate string (elt floor 1) "に着きました。"))
        (change-floor (format nil "~A/~a" (send *scene* :name)
                              (string-downcase (elt floor 0)))
                      *scene* *tfl*))))

  (unless (check-elevator-open :timeout 6)
    (send *ri* :go-pos-unsafe 0 0 -90)  ;; Unsafe for faster turning right
    (return-from ready-to-outside :fail))
  :success)
