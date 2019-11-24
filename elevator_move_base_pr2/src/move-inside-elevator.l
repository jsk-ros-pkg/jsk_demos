(load "package://elevator_move_base_pr2/src/utils.l")

(setq *change-publisher-topic*
      "/check_elevator_open/octree_change_publisher/octree_change_result")
(setq *change-threshold* (ros::get-param "change_threshold" 700))


(defun move-inside-state-machine ()
  (let ((sm (instance state-machine :init))
        (move-client (pr2-interface-move-base-action *ri*)))
    (send sm :add-node (instance state :init :set-param-for-elevator-outside
                                 'tolerance-loose))
    (send sm :add-node (actionlib-client-state
                        :move-elevator-front move-client :timeout 30 :retry nil
                        :key 'coords->movebaseactiongoal))
    (send (send sm :node :move-elevator-front)
          :remap-list '((:goal . elevator-front-coords)))
    (send sm :add-node (instance state :init :set-param-for-elevator-inside
                                 'tolerance-strict))
    (send sm :add-node
          (instance state :init :check-elevator-open
                    '(lambda (x) (check-elevator-open :timeout 300))))
    (send sm :add-node (actionlib-client-state
                        :move-elevator-inside move-client :timeout 40 :retry nil
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
    (send *ri* :angle-vector (send *pr2* :angle-vector) 400)
    (send *ri* :wait-interpolation)
    (setq *change-pc* nil)
    (ros::subscribe *change-publisher-topic* sensor_msgs::PointCloud2 #'pc-cb)
    (unix:usleep (* 300 1000))  ;; Wait until point cloud becomes stable

    (setq time-start (ros::time-now))
    (ros::rate 10)
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
    (unix:usleep (* 500 1000)) ;; wait until door open
    t))
