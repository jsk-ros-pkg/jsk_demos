
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-srvs "dynamic_tf_publisher")

(defun load-coords-from-db (db table parent child)
  (car (select-tf-from-table db :frame_id parent :child_id child :table table :limit 1)))

(defun transform-pose-to-target-frame (coords frame-id &optional (tm (ros::time 0))) ;; (send coords :name) is frame-if
  (let ((tra (send *tfl* :lookup-transform frame-id (send coords :name) tm)))
    (when tra
      (send tra :transform coords)
      (send tra :name frame-id)
      tra)))

(defun move-to (coords &key (retry 10))
  (let (cur cur2 diff (ret nil))
    (when (send *ri* :move-to coords :retry retry
		:frame-id (send coords :name)
		:wait-for-server-timeout 20)
      (setq cur (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)))
      (setq cur2 (send cur :copy-worldcoords))
      (setq diff (v- (send co :worldpos) (send cur :worldpos)))
      (setq diff (send (send cur2 :inverse-transformation) :rotate-vector diff))
      (when (< (norm diff) 500) ;; path length < 500mm
	(send *ri* :go-velocity (/ (elt diff 0) 3000) (/ (elt diff 1) 3000)
	      (/ (elt (car (rpy-angle (send (send cur :transformation co) :worldrot))) 0) 3)
	      3000) ;; 3sec
	(unix::sleep 3)
	(setq ret t)))
    ret ))

(defun check-current-floor (&optional pose) ;; :name == frame_id
  (if pose
      (setq pose (transform-pose-to-target-frame pose "/base_footprint"))
    (setq pose (send *tfl* :lookup-transform "/base_footprint" "/base_footprint" (ros::time 0))))
  (print (send-all (send *scene* :floors) :descendants))
  (print pose)
  (send *scene* :current-floor pose))

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
;; reset local costmap and clear unknown grid around robot
(defun switch-global-planner-observation (enable)
  (let ((obs (if enable
                 "base_scan_marking base_scan tilt_scan ground_object_cloud"
               "base_scan_marking base_scan tilt_scan")))
  (ros::set-param "/move_base_node/local_costmap/observation_sources" obs)
  ))
