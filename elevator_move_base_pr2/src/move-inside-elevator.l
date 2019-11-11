(setq *change-publisher-topic*
      "/check_elevator_open/octree_change_publisher/octree_change_result")
(setq *change-pc* nil)
(setq *change-threshold* (ros::get-param "change_threshold" 500))


(defun tolerance-loose (&rest args)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "xy_goal_tolerance" :double 0.20)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "yaw_goal_tolerance" :double 0.175)  ;; 10deg
  t)

(defun tolerance-strict (&rest args)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "xy_goal_tolerance" :double 0.10)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "yaw_goal_tolerance" :double 0.087)  ;; 5deg
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/global_costmap" "footprint_padding" :double 0.03)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/local_costmap" "footprint_padding" :double 0.03)
  (switch-global-planner-observation nil) ;;disable tilt scan
  t)

(defun move-inside-state-machine ()
  (let ((sm (instance state-machine :init))
        (move-client (pr2-interface-move-base-action *ri*)))
    (send sm :add-node (instance state :init :set-param-for-elevator-outside
                                 'tolerance-loose))
    (send sm :add-node (actionlib-client-state
                        :move-elevator-front move-client :timeout 300 :retry t
                        :key 'coords->movebaseactiongoal))
    (send (send sm :node :move-elevator-front)
          :remap-list '((:goal . elevator-front-coords)))
    (send sm :add-node (instance state :init :set-param-for-elevator-inside
                                 'tolerance-strict))
    (send sm :add-node
          (instance state :init :check-elevator-open 'check-elevator-open))
    (send sm :add-node (actionlib-client-state
                        :move-elevator-inside move-client :timeout 300 :retry t
                        :key 'coords->movebaseactiongoal))
    (send (send sm :node :move-elevator-inside)
          :remap-list '((:goal . inside-coords)))
    (send sm :arg-keys 'elevator-front-coords 'inside-coords)
    (send sm :goal-state (list :success :fail))
    (send sm :start-state :set-param-for-elevator-outside)
    (send sm :add-transition
          :set-param-for-elevator-outside :move-elevator-front t)
    (send sm :add-transition
          :move-elevator-front :set-param-for-elevator-inside :succeeded)
    (send sm :add-transition :move-elevator-front :fail :failed)
    (send sm :add-transition
          :set-param-for-elevator-inside :check-elevator-open t)
    (send sm :add-transition :check-elevator-open :move-elevator-inside t)
    (send sm :add-transition :check-elevator-open :fail nil)
    (send sm :add-transition :move-elevator-inside :success :succeeded)
    (send sm :add-transition :move-elevator-inside :fail :failed)
    sm))

(defun pc-cb (msg)
  (setq *change-pc* (* (send msg :height) (send msg :width)))
  (ros::ros-info "change: ~A" *change-pc*))

(defun check-elevator-open (&optional args)
  (ros::subscribe *change-publisher-topic* sensor_msgs::PointCloud2 #'pc-cb)
  (unix:usleep (* 500 1000))  ;; Wait until point cloud becomes stable
  (let ((start-time (ros::time-now)))
    (while (or
            (not *change-pc*)
            (< *change-pc* *change-threshold*))
      (ros::ros-warn "change: ~A, change-threshold: ~A"
                     *change-pc* *change-threshold*)
      (ros::spin-once)
      (ros::sleep)))
  (ros::unsubscribe *change-publisher-topic*)
  (unix:sleep 1) ;; wait until door open
  (clear-costmap)
  t)
