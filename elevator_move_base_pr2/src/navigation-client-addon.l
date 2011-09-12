#|
(progn (ros::roseus "hoge")
       (load "package://pr2eus/pr2-interface.l")
       (load "package://elevator_move_base_pr2/src/navigation-client-addon.l")
       (setq *ri* (instance pr2-interface :init)))
|#
(in-package "ROS")
(defmethod ros::simple-action-client
  (:goal-status-cb
   (msg)
   (if (string= name-space "move_base") (ros::ros-info "[~A] goal-status-cb ~A" name-space (send msg :status_list)))
   (if (send msg :status_list)
       (setf (get self :last-status-time) (ros::time-now)))
   (ros::ros-debug ";; status ~A~%" msg)
   (let ((m (send comm-state :find-status-by-goal-id msg)))
     (when m
       (let ((com-state (send comm-state :update-status msg))
             (error-msg
              (format nil "Received comm state ~A when in simple state ~A"
                      comm-state simple-state)))
         (case
             com-state
           ('ros::*comm-state-active*
             (cond ((= simple-state ros::*simple-goal-state-pending*)
                    (setq simple-state ros::*simple-goal-state-active*))
                   ((=  simple-state ros::*simple-goal-state-done*)
                    (ros::ros-error error-msg))))
           ('ros::*comm-state-recalling*
             (cond ((not (= simple-state ros::*simple-goal-state-pending*))
		    (ros::ros-error error-msg))))
           ('ros::*comm-state-preempting*
             (cond ((= simple-state ros::*simple-goal-state-pending*)
                    (setq simple-state ros::*simple-goal-state-active*))
                   ((=  simple-state ros::*simple-goal-state-done*)
                    (ros::ros-error error-msg))))
           ('ros::*comm-state-done*
             (cond ((member simple-state
                            (list ros::*simple-goal-state-pending*
                                  ros::*simple-goal-state-active*))
                    (setq simple-state ros::*simple-goal-state-done*))
                   )))
         ))))
  (:wait-for-result-and-check-status
   (&key (timeout 0)) ;; sec
   (let ((start (ros::time-now)))
     (ros::rate 10)
     (ros::ros-info "wait-for-goal-and-check-status")
     (while (ros::ok)
       (if (= simple-state ros::*simple-goal-state-done*)
	   (return))
       (if (> timeout 0)
	   (let* ((tm (ros::time- (ros::time-now) start)))
	     (if (> (send tm :to-sec) timeout) (return))))
       (ros::sleep)
       (ros::spin-once)
       (ros::ros-info "wait-for-result-and-check-status ~A"
		  (if (get self :last-status-time)
		      (send (ros::time- (ros::time-now) (get self :last-status-time) ) :to-sec)
		    nil))
       (unless (and (get self :last-status-time)
		    (< (send (ros::time- (ros::time-now) (get self :last-status-time) ) :to-sec) 5))
	 (return-from :wait-for-result-and-check-status nil))
       )
     (eq (send self :get-state) actionlib_msgs::GoalStatus::*succeeded*)))
  )

(in-package "USER")
(defmethod pr2-interface
  (:move-to
   (coords &key (retry 10) (frame-id "/map") (wait-for-server-timeout 5))
   (let (ret (count 0) (pose (ros::coords->tf-pose coords))
             (goal (instance move_base_msgs::MoveBaseActionGoal :init)))
     (when (not (send move-base-action :wait-for-server wait-for-server-timeout))
       (ros::ros-error "wait-for-server failed")
       (return-from :move-to))
     (send goal :goal :target_pose :header :frame_id frame-id)
     (send goal :goal :target_pose :pose pose)
     (ros::ros-error "ret = A~, count =~A, retry = ~A" ret count retry)
     (while (and (null ret) (<= (inc count) retry))
       (if (> count 0) ;; retry
	   (clear-costmap))
       (ros::ros-info "move-to : send-goal to (make-coords :pos ~A :rpy ~A) (~d)" (send coords :worldpos) (coerce (car (send coords :rpy-angle)) float-vector) count)
       (send move-base-action :send-goal goal)
       (send move-base-action :wait-for-result-and-check-status)
       (if (eq (send move-base-action :get-state) actionlib_msgs::GoalStatus::*succeeded*)
           (setq ret t)))
     (ros::ros-info "move-to : ~A" (if ret 'succeeded 'failed))
     ret))
  )

