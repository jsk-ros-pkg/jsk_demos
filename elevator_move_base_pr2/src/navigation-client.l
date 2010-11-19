
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-srvs "dynamic_tf_publisher")

(defun load-coords-from-db (db table parent child)
  (car (select-tf-from-table db :frame_id parent :child_id child :table table :limit 1)))

(if (and (boundp '*db*) *db*)
    (progn
      (warn "load spot from DB")
      ;; read current spots from DB
  ))

(defun transform-pose-to-target-frame (coords frame-id &optional (tm (ros::time 0))) ;; (send coords :name) is frame-if
  (let ((tra (send *tfl* :lookup-transform frame-id (send coords :name) tm)))
    (when tra
      (send tra :transform coords)
      (send tra :name frame-id)
      tra)))

(defun move-to (coords &key (retry 10))
  (send *ri* :move-to coords :retry retry
	:frame-id (send coords :name)
	:wait-for-server-timeout 20))

(defun check-current-floor (&optional pose)
  (if pose
      (setq pose (transform-pose-to-target-frame pose "/map"))
    (setq pose (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0))))
  (cond ((< (elt (send pose :pos) 0) -5000) "/eng8/1f")
	(t "/eng8/6f")))

(defun floor- (frame1 frame2)
  (let ((floor-list '("/eng8/7f" "/eng8/6f" "/eng8/5f" "/eng8/4f" "/eng8/3f" "/eng8/2f" "/eng8/1f" "/eng8/b1f" "/eng8/b2f")))
    (apply #'- (mapcar #'(lambda(f)(length (member f floor-list :test #'string=))) (list frame1 frame2)))
    ))

(defun change-floor (target-floor)
  (ros::advertise "/initialpose" geometry_msgs::PoseWithCovarianceStamped 1)
  (unix:sleep 1);; important
  (let* ((msg (instance geometry_msgs::PoseWithCovarianceStamped :init))
	 (current-floor-frame (check-current-floor))
	 (robot-pose (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)))
	 (trans (send *tfl*  :lookup-transform current-floor-frame target-floor (ros::time 0))))
    (send robot-pose :transform trans :world)
    (send msg :header :frame_id "/map")
    (send msg :header :stamp (ros::time-now))
    (let ((cov (send msg :pose :covariance)))
      (setf (elt cov 0) 0.05)
      (setf (elt cov 7) 0.05)
      (setf (elt cov 21) 0.02))
    (send msg :pose :pose (ros::coords->tf-pose robot-pose))
    (ros::publish "/initialpose" msg)))


;;
;; for dynamic configuration of costmap params
;;
(defun call-empty-service (srvname)
  (ros::wait-for-service srvname)
  (ros::service-call srvname (instance std_srvs::EmptyRequest :init)))

;; reset local costmap and clear unknown grid around robot
(defun clear-costmap ()
  (call-empty-service "/move_base_node/clear_local_costmap")
  (call-empty-service "/move_base_node/clear_unknown_space"))

(defun initialize-costmaps ()
  (call-empty-service "move_base_node/global_costmap/initialize_costmap")
  (call-empty-service "move_base_node/local_costmap/initialize_costmap")
  (clear-costmap))

(defun change-inflation-range (&optional (range 0.55))
  (if (eps= (ros::get-param "/move_base_node/local_costmap/inflation_radius")
	    range 0.1)
      (return-from change-inflation-range nil))
  (ros::set-param "/move_base_node/global_costmap/inflation_radius" range)
  (ros::set-param "/move_base_node/local_costmap/inflation_radius" range)
  )

(defun switch-global-planner-observation (enable)
  (let ((obs (if enable
                 "base_scan_marking base_scan tilt_scan ground_object_cloud"
               "base_scan_marking base_scan tilt_scan")))
  (ros::set-param "/move_base_node/local_costmap/observation_sources" obs)
  (ros::set-param "/move_base_node/global_costmap/observation_sources" obs)
  ))
