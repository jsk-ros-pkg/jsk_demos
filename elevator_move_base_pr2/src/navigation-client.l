
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-srvs "dynamic_tf_publisher")

(defun load-coords-from-db (db table parent child)
  (car (select-tf-from-table db :frame_id parent :child_id child :table table)))

(if (and (boundp '*db*) *db*)
    (progn
      (warn "load spot from DB")
      ;; read current spots from DB
      (setq *eng8-room602-front*
	    (load-coords-from-db *db* "spot" "/eng8" "/eng8-room602-front"))
      (setq *eng8-elevator-panel-front*
	    (load-coords-from-db *db* "spot" "/eng8" "/eng8-elevator-panel-front"))
      (setq *eng8-elevator-front*
	    (load-coords-from-db *db* "spot" "/eng8" "/eng8-elevator-front"))
      (setq *eng8-elevator-outside*
	    (load-coords-from-db *db* "spot" "/eng8" "/eng8-elevator-outside"))
      (setq *eng8-elevator-inside*
	    (load-coords-from-db *db* "spot" "/eng8" "/eng8-elevator-inside"))
      (setq *eng8-pamphlet-front*
	    (load-coords-from-db *db* "spot" "/eng8" "/eng8-pamphlet-front"))
      )
  (progn
    (warn "*db* is not bounded")
    (setq *eng8-room602-front*
	  (make-coords :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
		       :pos #f(-400.0 100.0 0.0)))
    (setq *eng8-elevator-panel-front*
	  (make-coords :rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
		       :pos #f(3200.0 -29100.0 0.0)))
    (setq *eng8-elevator-front*
	  (make-coords :rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
		       :pos #f(4000.0 -29000.0 0.0)))
    (setq *eng8-elevator-outside*
	  (make-coords :rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
		       :pos #f(4000.0 -29000.0 0.0)))
    (setq *eng8-elevator-inside*
	  (make-coords :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
		       :pos #f(4100.0 -31300.0 0.0)))
    (setq *eng8-pamphlet-front*
	  (make-coords :rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
		       :pos #f(15500.0 -33000.0 0.0))))
  )

;(defun move-to (coords &key (retry 10))
;  (dotimes (i retry)
;    (let ((co (send coords :copy-worldcoords))
;	  (randomv (random-vector 100)))
;      (send co :translate randomv)
;      (when (send *ri* :move-to co :retry 1
;		  :frame-id (floor-frame-id (check-current-floor))
;		  :wait-for-server-timeout 20)
;      (return-from move-to t)))
;  nil))

(defun transform-pose-to-target-frame (coords frame-id &optional (tm (ros::time 0))) ;; (send coords :name) is frame-if
  (let ((tra (send *tfl* :lookup-transform frame-id (send coords :name) tm)))
    (when tra
      (send tra :transform coords)
      (send tra :name frame-id)
      tra)))

(defun move-to (coords &key (retry 10))
  (send *ri* :move-to coords :retry retry
	:frame-id (check-current-floor)
	:wait-for-server-timeout 20))

(defun check-current-floor (&optional pose)
  (if pose
      (setq pose (transform-pose-to-target-frame pose "/map"))
    (setq pose (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0))))
  (cond ((< (elt (send pose :pos) 0) -5000) "/eng8_1f")
	(t "/eng8_6f")))

(defun floor- (frame1 frame2)
  (let ((floor-list '("/eng8_7f" "/eng8_6f" "/eng8_5f" "/eng8_4f" "/eng8_3f" "/eng8_2f" "/eng8_1f" "/eng8_b1f" "/eng8_b2f")))
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
;  (ros::publish "robotsound"
;		(instance sound_play::SoundRequest :init
;			  :sound sound_play::needs_unplugging
;			  :command sound_play::play_once))
  )

(defun switch-global-planner-observation (enable)
  (let ((obs (if enable
		 "base_scan_marking base_scan tilt_scan ground_object_cloud"
	       "base_scan_marking base_scan tilt_scan")))
  (ros::set-param "/move_base_node/local_costmap/observation_sources" obs)
  (ros::set-param "/move_base_node/global_costmap/observation_sources" obs)
  ))


;; when input point cloud isn't publishing
;; move_base doesn't work
;; -> send base_scan as dummy tilt_scan
(defun tilt-scan->costmap (&key (enable t))
  (let ((srvname "/tilt_laser_mux/select")
	(req (instance topic_tools::MuxSelectRequest :init
		       :topic (if enable "/ground_object_cloud" "/empty_cloud"))))
    (ros::wait-for-service srvname)
    (send (ros::service-call srvname req) :prev_topic)
    ))



;; old functions
(defun -change-map-old (mapname)
  (ros::wait-for-service "set_dynamic_tf")
  (setq req (instance dynamic_tf_publisher::SetDynamicTFRequest :init))
  (send req :freq 20)
  (send req :cur_tf
	(ros::coords->tf-transform-stamped (make-coords) mapname "/map"))
  (ros::service-call "set_dynamic_tf" req)
  )
(defun -change-inflation-range-old (&optional (range 0.55))
  (if (eps= (ros::get-param "/move_base_node/local_costmap/inflation_radius")
	    range 0.1)
      (return-from change-inflation-range nil))

  (ros::publish "robotsound"
		(instance sound_play::SoundRequest :init
			  :sound sound_play::needs_unplugging
			  :command sound_play::play_once))
  (ros::set-param "/move_base_node/local_costmap/inflation_radius" range)
  (unix::system "rosnode kill /move_base_node")
  (unix::sleep 1)
  )
