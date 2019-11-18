(setq *change-publisher-topic*
      "/check_elevator_open/octree_change_publisher/octree_change_result")
(setq *change-threshold* (ros::get-param "change_threshold" 700))


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
  (ros::ros-info "point cloud change: ~A" *change-pc*))

(defun check-elevator-open (&key (timeout 300))
  (let (time-start)
    (send *pr2* :head :angle-vector #f(0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (setq *change-pc* nil)
    (ros::subscribe *change-publisher-topic* sensor_msgs::PointCloud2 #'pc-cb)
    (unix:sleep 1)  ;; Wait until point cloud becomes stable

    (setq time-start (ros::time-now))
    (ros::rate 5)
    (while (or
            (not *change-pc*)
            (< *change-pc* *change-threshold*))
      (when (ros::time> (ros::time-now)
                        (ros::time+ time-start (ros::time timeout)))
        (ros::warn
         (format nil "Could not detect door opening in ~A [sec]." timeout))
        (speak-jp "ドアが開いていません。")
        (return-from check-elevator-open nil))
      (ros::ros-warn "point cloud change: ~A, threshold of open/closed: ~A"
                     *change-pc* *change-threshold*)
      (ros::spin-once)
      (ros::sleep))
    (ros::unsubscribe *change-publisher-topic*)
    (ros::ros-info "Door opened.")
    (speak-jp "ドアが開きました。")
    (clear-costmap)
    (unix:sleep 2) ;; wait until door open
    t))
