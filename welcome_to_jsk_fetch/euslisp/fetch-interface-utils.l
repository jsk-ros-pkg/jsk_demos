(load "package://fetcheus/fetch-interface.l")

(defmethod fetch-interface
    (:move-to-wait
     (&rest args &key (retry 10) (frame-id "world") (correction t) (wait-for-result-duration-time 0) &allow-other-keys)
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
                     ;;
                     (when (and ret correction)
                       (let (diff diff-len current-coords lret map-goal-coords)
                         ;;
                         (setq map-goal-coords
                               (if (string= frame-id base-frame-id)
                                   (send (send map-to-frame :copy-worldcoords) :transform (send coords :worldcoords))
                                   (send (send *tfl* :lookup-transform "map" frame-id (ros::time 0))
                                         :transform (send coords :copy-worldcoords)))) ;; goal-coords in /map coordinates
                         (setq lret (send *tfl* :wait-for-transform "map" base-frame-id (ros::time-now) 5))
                         (ros::ros-warn ":move-to wait-for transform map to ~A -> ~A" base-frame-id lret)
                         (when (null lret)
                           (ros::ros-error ":move-to wait-for transform map to ~A failed" base-frame-id)
                           (setq move-base-goal-msg nil)
                           (return-from :move-to-wait nil))
                         (setq current-coords (send *tfl* :lookup-transform "map" base-frame-id (ros::time 0)))
                         (setq diff (send current-coords :transformation map-goal-coords))
                         (ros::ros-warn ":move-to current-coords  ~A" current-coords)
                         (ros::ros-warn "         mapgoal-coords  ~A" map-goal-coords)
                         (ros::ros-warn "          error-coords   ~A" diff)
                         (ros::ros-warn "         target-coords   ~A" coords)
                         ;;
                         (dotimes (i 2)
                           (if (< (setq diff-len (norm (subseq (send diff :worldpos) 0 2))) 200) ;; move_base thre = 200mm
                               (let* ((msec (* diff-len 10))
                                      (x (/ (elt (send diff :worldpos) 0) msec))
                                      (y (/ (elt (send diff :worldpos) 1) msec))
                                      (d (/ (elt (car (rpy-angle (send diff :worldrot))) 0) (/ msec 1000))))
                                 (ros::ros-warn ":move-to -> :go-velocity x:~A y:~A d:~A msec:~A" x y d msec)
                                 (unix:usleep (* 400 1000)) ;; 400ms ???
                                 (let ((acret (send self :go-velocity x y d msec :wait t)))
                                   (unless acret
                                     (setq move-base-goal-msg nil)
                                     (return-from :move-to-wait nil)))
                                 ;;(unix::usleep (* (round msec) 1000)) ;; why time wait
                                 )
                               (progn
                                 (ros::ros-error "too far from goal position ~A mm (> 200mm)" diff-len)
	                               ;; move-to succeeded but away from 200 mm
	                               (ros::ros-error ":move-to try to send /move_base_simple/goal")
	                               (ros::advertise "/move_base_simple/goal" geometry_msgs::PoseStamped 1)
	                               (send move-base-goal-msg :goal :target_pose :header :seq (1+ count))
	                               (ros::publish "/move_base_simple/goal" (send move-base-goal-msg :goal :target_pose))
	                               (unix:sleep 3)
                                 (setq move-base-goal-msg nil)
                                 (return-from :move-to-wait nil)
                                 ))
                           ;;
                           (setq map-goal-coords
                                 (if (string= frame-id base-frame-id)
                                     (send (send map-to-frame :copy-worldcoords) :transform (send coords :worldcoords))
                                     (send (send *tfl* :lookup-transform "map" frame-id (ros::time 0))
                                           :transform (send coords :copy-worldcoords)))) ;; goal-coords in /map coordinates
                           (setq lret (send *tfl* :wait-for-transform "map" base-frame-id (ros::time-now) 5))
                           (ros::ros-warn ":move-to wait-for transform map to ~A -> ~A" base-frame-id lret)
                           (when (null lret)
                             (ros::ros-error ":move-to wait-for transform map to ~A failed" base-frame-id)
                             (setq move-base-goal-msg nil)
                             (return-from :move-to-wait nil))
                           (setq current-coords (send *tfl* :lookup-transform "map" base-frame-id (ros::time 0)))
                           (setq diff (send current-coords :transformation map-goal-coords))
                           (ros::ros-warn ":move-to current-coords  ~A" current-coords)
                           (ros::ros-warn "         mapgoal-coords  ~A" map-goal-coords)
                           (ros::ros-warn "          error-coords   ~A" diff)
                           (ros::ros-warn "         target-coords   ~A" coords)
                           ) ;;  (do (i 2)
                         ))
                     (setq move-base-goal-msg nil) ;; :move-to-wait has been called
                     ret)))
