(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-srvs "std_srvs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(load "package://elevator_move_base_pr2/src/utils.l")


(defun speak-to-human-behind (userdata)
  (let (start-tm (timeout (ros::time 10))
        (sub-inside-p nil) (sub-outside-p nil) (inside-p nil) (outside-p nil))
    (ros::ros-info "speak-to-human-behind: start")
    (call-service "/human_in_mirror/pass_through_kinect_rgb/request"
                  (instance std_srvs::EmptyRequest :init))
    (call-service "/human_in_mirror/pass_through_kinect_depth/request"
                  (instance std_srvs::EmptyRequest :init))
    (setq start-tm (ros::time-now))

    (ros::subscribe
     "/human_in_mirror/find_human_in_mirror/output/inside_mirror"
     jsk_recognition_msgs::BoolStamped
     #'(lambda (m)
         (when (ros::time< start-tm (send m :header :stamp))
           (setq sub-inside-p t)
           (setq inside-p (send m :data)))))
    (ros::subscribe
     "/human_in_mirror/find_human_in_mirror/output/outside_mirror"
     jsk_recognition_msgs::BoolStamped
     #'(lambda (m)
         (when (ros::time< start-tm (send m :header :stamp))
           (setq sub-outside-p t)
           (setq outside-p (send m :data)))))

    (while (and (ros::ok)
                (not (and sub-inside-p sub-outside-p))
                (ros::time< (ros::time-now) (ros::time+ start-tm timeout)))
      (ros::spin-once)
      (when (and sub-inside-p sub-outside-p)
        ;; Speak fast!
        (cond ((and inside-p outside-p)
               (ros::ros-info
                "Speaking to human behind robot and inside elevator...")
               (speak-jp "後ろの方お先に失礼します、中の方お邪魔します。"
                         :wait nil))
              ((and inside-p (not outside-p))
               (ros::ros-info "Speaking to human behind robot...")
               (speak-jp "後ろの方お先に失礼します。" :wait nil))
              ((and (not inside-p) outside-p)
               (ros::ros-info "Speaking to human inside elevator...")
               (speak-jp "エレベータ内の方お邪魔します。" :wait nil))
              (t
               (ros::ros-info "Nobody was found. Skip speaking."))))
      (ros::sleep))
    (call-service "/human_in_mirror/pass_through_kinect_rgb/stop"
                  (instance std_srvs::EmptyRequest :init))
    (call-service "/human_in_mirror/pass_through_kinect_depth/stop"
                  (instance std_srvs::EmptyRequest :init))
    (ros::unsubscribe "/human_in_mirror/find_human_in_mirror/output")
    (ros::ros-info "speak-to-human-behind: end")
    t))
