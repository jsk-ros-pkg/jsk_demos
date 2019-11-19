(ros::roseus-add-msgs "geometry_msgs")


(defun load-coords-from-db (db table parent child)
  (car (select-tf-from-table
        db :frame_id parent :child_id child :table table :limit 1)))

(defun transform-pose-to-target-frame (coords frame-id
                                       &optional (tm (ros::time 0)))
  (let ((tra (send *tfl* :lookup-transform frame-id (send coords :name) tm)))
    (when tra
      (send tra :transform coords)
      (send tra :name frame-id)
      tra)))

(defun check-current-floor (&optional pose) ;; :name == frame_id
  (if pose
      (setq pose (transform-pose-to-target-frame pose (send *scene* :name)))
    (setq pose (send *tfl* :lookup-transform
                     (send *scene* :name) "base_footprint" (ros::time 0))))
  (send *scene* :current-floor pose))

(defun change-floor (target-floor &optional (topicname "/initialpose3d"))
  (ros::advertise topicname geometry_msgs::PoseWithCovarianceStamped 1)
  (unix:sleep 1);; important
  (let* ((msg (instance geometry_msgs::PoseWithCovarianceStamped :init))
         (current-floor-frame (check-current-floor))
         (robot-pose (send *tfl* :lookup-transform
                           current-floor-frame "base_footprint" (ros::time 0))))
    (if (null robot-pose)
        (return-from change-floor nil))
    (send msg :header :frame_id target-floor)
    (send msg :header :stamp (ros::time-now))
    (let ((cov (send msg :pose :covariance)))
      (setf (elt cov 0) 0.05)
      (setf (elt cov 7) 0.05)
      (setf (elt cov 21) 0.02))
    (send msg :pose :pose (ros::coords->tf-pose robot-pose))
    (ros::publish topicname msg)))

;; for dynamic configuration of costmap params
(defun switch-global-planner-observation (enable)
  (use-tilt-laser-obstacle-cloud enable))
