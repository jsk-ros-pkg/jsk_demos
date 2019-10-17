(setq *change-publisher-topic* "/pcl_nodelet/octree_change_detector/octree_change_result")
(setq *change-pc* nil)
(setq *change-threshold* (ros::get-param "change_threshold" 200))

(defun move-inside-state-machine ()
  (let ((sm (instance state-machine :init))
        (move-client (pr2-interface-move-base-action *ri*)))
    (send sm :add-node (instance state :init :tuckarm-pose '(lambda (&optional args) (pr2-tuckarm-pose))))
    (send sm :add-node (actionlib-client-state
                        :move-elevator-front move-client :timeout 300 :retry t
                        :key 'coords->movebaseactiongoal))
    (send (send sm :node :move-elevator-front) :remap-list '((:goal . front-coords)))
    (send sm :add-node (instance state :init :check-elevator-open 'check-elevator-open))
    (send sm :add-node (actionlib-client-state
                        :move-elevator-inside move-client :timeout 300 :retry t
                        :key 'coords->movebaseactiongoal))
    (send (send sm :node :move-elevator-inside) :remap-list '((:goal . inside-coords)))
    (send sm :arg-keys 'front-coords 'inside-coords)
    (send sm :goal-state (list :success :fail))
    (send sm :start-state :tuckarm-pose)
    (send sm :add-transition :tuckarm-pose :move-elevator-front t)
    (send sm :add-transition :move-elevator-front :check-elevator-open :succeeded)
    (send sm :add-transition :move-elevator-front :fail :failed)
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
