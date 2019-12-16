(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-srvs "std_srvs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(load "package://elevator_move_base_pr2/src/utils.l")


(defun speak-to-human-behind (userdata)
  (let (start-tm (timeout (ros::time 8))
        (sub-inside-num 0) (sub-outside-num 0) (sub-req 1)
        (inside-p nil) (outside-p nil))
    (call-service "/human_in_mirror/pass_through_kinect_rgb/request"
                  (instance std_srvs::EmptyRequest :init))
    (call-service "/human_in_mirror/pass_through_kinect_depth/request"
                  (instance std_srvs::EmptyRequest :init))
    (ros::subscribe
     "/human_in_mirror/find_human_in_mirror/output/inside_mirror"
     jsk_recognition_msgs::BoolStamped
     #'(lambda (m)
         (when (ros::time< start-tm (send m :header :stamp))
           (incf sub-inside-num)
           (setq inside-p (send m :data)))))  ;; Use only latest data
    (ros::subscribe
     "/human_in_mirror/find_human_in_mirror/output/outside_mirror"
     jsk_recognition_msgs::BoolStamped
     #'(lambda (m)
         (when (ros::time< start-tm (send m :header :stamp))
           (incf sub-outside-num)
           (setq outside-p (send m :data)))))  ;; Use only latest data

    (setq start-tm (ros::time-now))
    (while (and (ros::ok)
                (or (< sub-inside-num sub-req) (< sub-outside-num sub-req))
                (ros::time< (ros::time-now) (ros::time+ start-tm timeout)))
      (ros::spin-once)
      (when (and (>= sub-inside-num sub-req) (>= sub-outside-num sub-req))
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
               (ros::ros-info "Nobody was found. Move into elevator.")
               (speak-jp "エレベータに入ります。" :wait nil))))
      (ros::sleep))
    (call-service "/human_in_mirror/pass_through_kinect_rgb/stop"
                  (instance std_srvs::EmptyRequest :init))
    (call-service "/human_in_mirror/pass_through_kinect_depth/stop"
                  (instance std_srvs::EmptyRequest :init))
    (ros::unsubscribe
     "/human_in_mirror/find_human_in_mirror/output/inside_mirror")
    (ros::unsubscribe
     "/human_in_mirror/find_human_in_mirror/output/outside_mirror")
    t))
