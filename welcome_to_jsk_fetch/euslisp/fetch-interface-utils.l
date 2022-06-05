(load "package://fetcheus/fetch-interface.l")

(defmethod fetch-interface
    (:move-to-wait-once
     (&rest args &key (retry 0) (frame-id "world") (wait-for-result-duration-time 0) &allow-other-keys)
                   (let (ret (count 0) (tm (ros::time-now))
	                           (map-to-frame move-base-goal-map-to-frame)
                             (coords move-base-goal-coords))
                     (when (send self :simulation-modep)
                       ;; wait for-result
                       (while current-goal-coords
                         (send self :robot-interface-simulation-callback))
                       (return-from :move-to-wait t)) ;; simulation-modep
                     (if (null move-base-goal-msg) (return-from :move-to-wait nil))
                     (while (and (null ret) (<= count retry))
                       (when (> count 0) ;; retry
                         (send self :clear-costmap)
                         (send move-base-goal-msg :header :seq count)
                         (send move-base-goal-msg :goal :target_pose :header :seq count)
                         (ros::ros-info "move-to : send-goal to ~A at ~A (~d)" (ros::tf-point->pos (send move-base-goal-msg :goal :target_pose :pose :position)) (send move-base-goal-msg :goal :target_pose :header :frame_id) count)
                         (send move-base-action :send-goal move-base-goal-msg))
                       (send move-base-action :wait-for-result :timeout wait-for-result-duration-time)
                       (ros::ros-info "get-state ~A" (send move-base-action :get-state))
                       (when (eq (send move-base-action :get-state)
                                 actionlib_msgs::GoalStatus::*preempted*)
                         (setq ret nil)
                         (return))
                       (if (eq (send move-base-action :get-state) actionlib_msgs::GoalStatus::*succeeded*)
                           (setq ret t))
                       (incf count))
                     (ros::ros-info "move-to : ~A" (if ret 'succeeded 'failed))
                     ret)))
