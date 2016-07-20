(require :pr2-interface "package://pr2eus/pr2-interface.l")

(ros::roseus "wander")
(unless (boundp '*ri*) (pr2-init))

(defvar *move-base-counter* 0)
(defvar *base-frame-id* "/base_footprint")
(unless (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))

(defun coords->movebaseactiongoal (co &optional (map "/map") (frame "/world") (stamp (ros::time-now)))
  (let ((goal (instance move_base_msgs::movebaseactiongoal :init))
        map->frame)
    (if (send *tfl* :wait-for-transform map frame (ros::time 0) 3)
        (setq map->frame (send *tfl* :lookup-transform map frame (ros::time 0)))
      (setq map->frame (make-coords)))
    (send goal :header :seq (inc *move-base-counter*))
    (send goal :header :stamp stamp)
    (send goal :header :frame_id map)
    (send goal :goal :target_pose :header :seq *move-base-counter*)
    (send goal :goal :target_pose :header :stamp stamp)
    (send goal :goal :target_pose :header :frame_id map)
    (send goal :goal :target_pose :pose
          (ros::coords->tf-pose (send (send co :copy-worldcoords) :transform map->frame :world)))
    goal))

(defun move-to-with-timeout (cds
                             &key
                             (map "/map")
                             (frame-id "/world")
                             (timeout 300) ;; [sec]
                             (clear-costmap) ;; clear before move
                             (max-retry-count 3))
  (let ((ac (pr2-interface-move-base-action *ri*))
        (retry-count 0)
        (start-time (ros::time-now)))
    (send ac :wait-for-server)
    (when clear-costmap
      (clear-costmap))
    (send ac :send-goal (coords->movebaseactiongoal cds map frame-id))
    (ros::ros-info "send-goal ~A -> ~A" (send *ri* :state :worldcoords) cds)
    (send ac :spin-once)
    (while (ros::ok)
      (cond
       ((eq (send ac :get-state) actionlib_msgs::GoalStatus::*SUCCEEDED*)
        (return-from move-to-with-timeout t))
       ((eq (send ac :get-state) actionlib_msgs::GoalStatus::*ABORTED*)
        (cond
         ((<= (inc retry-count) max-retry-count)
          (ros::ros-warn "retrying ~A/~A (move-to ~A)" retry-count max-retry-count cds)
          (setq start-time (ros::time-now))
          (clear-costmap)
          (send ac :send-goal (coords->movebaseactiongoal cds map frame-id)))
         (t (ros::ros-error "failed after retrying")
            (return))))
       ((member (send ac :get-state)
                (list actionlib_msgs::GoalStatus::*PREEMPTED*
                      actionlib_msgs::GoalStatus::*RECALLED*
                      actionlib_msgs::GoalStatus::*REJECTED*))
        (ros::ros-warn "aborted")
        (return-from move-to-with-timeout nil))
       ((< timeout (send (ros::time- (ros::time-now) start-time) :to-sec))
        (ros::ros-warn "timeout cancel goal...")
        (send ac :cancel-goal)
        (return-from move-to-with-timeout nil))
       (t
        (send ac :spin-once)
        (ros::sleep))))
    (ros::ros-error "failure (move-to ~A)" cds)
    (return-from move-to-with-timeout nil)))

(defun go-pos-with-timeout (x y th
                              &key (timeout 300) (clear-costmap) (max-retry-count 3))
  (let ((cds (make-coords :pos (float-vector (* 1000 x) (* 1000 y) 0)
                          :rpy (float-vector (deg2rad th) 0 0))))
    (move-to-with-timeout cds
                          :map *base-frame-id*
                          :frame-id *base-frame-id*
                          :timeout timeout
                          :clear-costmap clear-costmap
                          :max-retry-count max-retry-count)))
                          
(defun look-around-and-wait (&key (random-neck-p t))
  (send *pr2* :head :neck-p :joint-angle
        (if random-neck-p (+ 15 (random-gauss 15)) 0))
  (let* ((div 6)
         (step-angle (/ (- (send *pr2* :head :neck-y :max-angle)
                           (send *pr2* :head :neck-y :min-angle))
                        div)))
    (dotimes (i div)
      (send *pr2* :head :neck-p :joint-angle
            (if random-neck-p (+ 15 (random-gauss 15)) 0))
      (send *pr2* :head :neck-y :joint-angle
            (+ (send *pr2* :head :neck-y :min-angle)
               (* i step-angle)))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      (send *ri* :wait-interpolation :timeout 1)
      (unix:sleep 3))))

(defun wander ()
  (let ((rand-x (random 1.0))
        (rand-y (random 1.0))
        (rand-th (random 360))
        (before-cds (send *ri* :state :worldcoords))
        after-cds)
    (while (null before-cds)
      (setq before-cds (send *ri* :state :worldcoords))
      (send *ri* :spin-once))
    (go-pos-with-timeout rand-x rand-y rand-th :timeout 5)
    (while (null after-cds)
      (setq after-cds (send *ri* :state :worldcoords))
      (send *ri* :spin-once))
    (when (< (norm (send after-cds :difference-position before-cds)) 10)
      (send *ri* :go-pos-unsafe 0 0 90)))
  (look-around-and-wait))
    
(defun wander-init ()
  (pr2-init)
)

#|
(init)
(ros::rate 1)

(do-until-key
 (let (prev-pos)
   (wander)
   (ros::spin-once)
   (ros::sleep))
|#

(provide :wander-pr2)
