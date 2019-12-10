(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-srvs "std_srvs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(load "package://elevator_move_base_pr2/src/utils.l")


(defun speak-to-human-behind (userdata)
  (let (start-tm (timeout (ros::time 10)) (spoke-p nil))
    (call-service "/human_in_mirror/pass_through_kinect_rgb/request"
                  (instance std_srvs::EmptyRequest :init))
    (call-service "/human_in_mirror/pass_through_kinect_depth/request"
                  (instance std_srvs::EmptyRequest :init))
    (ros::subscribe "/human_in_mirror/find_human_in_mirror/output"
                    jsk_recognition_msgs::BoolStamped
                    #'(lambda (m)
                        (when (send m :data)
                          (ros::ros-info "Speaking to human behind robot...")
                          (speak-jp "後ろの方、お先に失礼します。" :wait nil))
                        (setq spoke-p t)))
    (setq start-tm (ros::time-now))
    (while
        (and (ros::ok)
             (not spoke-p)
             (ros::time< (ros::time-now) (ros::time+ start-tm timeout)))
      (ros::spin-once))
    (call-service "/human_in_mirror/pass_through_kinect_rgb/stop"
                  (instance std_srvs::EmptyRequest :init))
    (call-service "/human_in_mirror/pass_through_kinect_depth/stop"
                  (instance std_srvs::EmptyRequest :init))
    (ros::unsubscribe "/human_in_mirror/find_human_in_mirror/output")
    t))
