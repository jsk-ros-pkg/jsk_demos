#!/usr/bin/env roseus
;; look-on-attention.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "interactive_behavior_201409")
(ros::load-ros-manifest "move_base_msgs")

(ros::roseus "idle_behavior")

(defparameter *tfl* (instance ros::transform-listener :init))
(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defparameter *last-behavior-time* (ros::time 0))
(defparameter *pending-attention* nil)
(defparameter *look-at-enabled* t)

(defparameter *attention-topic* "/interaction/attention")
(defparameter *base-frame-id* (ros::get-param "~base_frame_id" "base_footprint"))
(defparameter *sensor-frame-id* (ros::get-param "~sensor_frame_id" "high_def_frame"))
(defparameter *look-at-interval* (ros::get-param "~look_at_interval" 10.0))

;; for base movement
(defparameter *enable-base-movement* (ros::get-param "~use_base" t))
(defparameter *move-base-interval* (ros::get-param "~move_base_interval" 180.0))
(defparameter *last-base-movement* (ros::time 0))

;; suppress print violate max/min-angle
(unless (assoc :joint-angle-org (send rotational-joint :methods))
  (rplaca (assoc :joint-angle (send rotational-joint :methods)) :joint-angle-org)
  (defmethod rotational-joint
      (:joint-angle
       (&optional v &key relative &allow-other-keys)
       (let ()
         (when v
           (when (and joint-min-max-table joint-min-max-target)
             (setq min-angle (send self :joint-min-max-table-min-angle)
                   max-angle (send self :joint-min-max-table-max-angle)))
           (if relative (setq v (+ v joint-angle)))
           (cond ((> v max-angle)
                  (setq v max-angle)))
           (cond ((< v min-angle)
                  (setq v min-angle)))
           (setq joint-angle v)
           (send child-link :replace-coords default-coords)
           (send child-link :rotate (deg2rad joint-angle) axis))
         joint-angle)))
  ) ;; defmethod rotational-joint
(unless (assoc :joint-angle-org (send linear-joint :methods))
  (rplaca (assoc :joint-angle (send linear-joint :methods)) :joint-angle-org)
  (defmethod linear-joint
      (:joint-angle
       (&optional v &key relative &allow-other-keys)
       "return joint-angle if v is not set, if v is given, set joint angle. v is linear value in [mm]."
       (let ()
         (when v
           (if relative (setq v (+ v joint-angle)))
           (when (> v max-angle)
             (setq v max-angle))
           (when (< v min-angle)
             (setq v min-angle))
           (setq joint-angle v)
           (send child-link :replace-coords default-coords)
           (send child-link :translate (scale joint-angle axis)))
         joint-angle))))

(defun coords->movebaseactiongoal (cds)
  (let ((goal (instance move_base_msgs::MoveBaseActionGoal :init))
        (stamp (ros::time-now)))
    (when (or (null (send cds :name)) (null-string-p (send cds :name)))
      (ros::ros-error "coords must have valid frame_id")
      (return-from coords->movebaseactiongoal nil))
    (send goal :header :stamp stamp)
    (send goal :header :frame_id (send cds :name))
    (send goal :goal :target_pose :header :stamp stamp)
    (send goal :goal :target_pose :header :frame_id (send cds :name))
    (send goal :goal :target_pose :pose
          (ros::coords->tf-pose (send cds :copy-worldcoords)))
    goal))

(defun move-to-with-timeout (cds
                             &key (timeout 5) (clear-costmap) (max-retry 3) (action-client))
  "the same as (send *ri* :move-to cds), but with timeout
   Args:
    cds: target coords (name must be the frame id relative to)
    action-client: action client that sends goal/cancel
    timeout: timeout in secords that cancel command is published
    clear-costmap: clear costmap before base movement
    max-retry-count: max retry count to send goal to move_base"
  (unless action-client
    (setq action-client (robot-move-base-interface-move-base-action *ri*)))
  (let ((retry 0) (start-time (ros::time-now)) goal)
    (send action-client :wait-for-server)
    (when clear-costmap (clear-costmap :node-name "move_base"))
    (setq goal (coords->movebaseactiongoal cds))
    (unless goal (return-from move-to-with-timeout nil))
    (send action-client :send-goal goal)
    (ros::ros-info "goal: ~A -> ~A" (send *ri* :state :worldcoords) cds)
    (send action-client :spin-once)
    (while (ros::ok)
      (let ((s (send action-client :get-state)))
        (cond
          ((eq s actionlib_msgs::GoalStatus::*SUCCEEDED*)
           (return-from move-to-with-timeout t))
          ((eq s actionlib_msgs::GoalStatus::*ABORTED*)
           (cond
             ((<= (inc retry) max-retry)
              (ros::ros-warn "retry ~A/~A (move-to ~A)" retry max-retry cds)
              (setq start-time (ros::time-now))
              (clear-costmap :node-name "move_base")
              (setq goal (coords->movebaseactiongoal cds))
              (unless goal (return))
              (send action-client :send-goal goal))
             (t
              (ros::ros-error "failed after retrying")
              (return))))
          ((member s (list actionlib_msgs::GoalStatus::*PREEMPTED*
                           actionlib_msgs::GoalStatus::*RECALLED*
                           actionlib_msgs::GoalStatus::*REJECTED*))
           (ros::ros-warn "aborted")
           (return))
          ((< timeout (send (ros::time- (ros::time-now) start-time) :to-sec))
           (ros::ros-warn "timed out. sending cancel")
           (return))
          (t
           (send action-client :spin-once)
           (ros::sleep)))))
    ;; reaches here only if robot did not reach the goal
    (ros::ros-error "failed (move-to ~A)" cds)
    (send action-client :cancel-goal)
    (send action-client :wait-for-result
          :timeout timeout :return-if-server-down t :wait-rate 10)
    nil))

(defun go-pos-with-timeout (x y th
                            &key (timeout 5) (clear-costmap) (max-retry 3) (action-client))
  "the same as (send *ri* :go-pos x y th), but with timeout
   Args:
    x, y, th: target relative pose
    action-client: action client that sends goal/cancel
    timeout: timeout in secords that cancel command is published
    clear-costmap: clear costmap before base movement
    max-retry-count: max retry count to send goal to move_base"
  (let ((cds (make-cascoords :name *base-frame-id*)))
    (send cds :translate (float-vector (* 1000 x) (* 1000 y) 0))
    (send cds :rotate (deg2rad th) :z)
    (move-to-with-timeout cds
                          :timeout timeout
                          :clear-costmap clear-costmap
                          :max-retry max-retry
                          :action-client action-client)))

(defun wander (&key
                 (max-x 1.0) (max-y 1.0) (max-th 360)
                 (timeout 10) (clear-costmap t) (stuck-thresh 100))
  (let (init-cds goal-cds diff-pos)
    (while (null init-cds)
      (unless (ros::ok) (return-from wander nil))
      (setq init-cds (send *ri* :state :worldcoords))
      (send *ri* :spin-once))
    (go-pos-with-timeout
     (random max-x) (random max-y) (random max-th)
     :clear-costmap clear-costmap
     :timeout timeout)
    (while (null goal-cds)
      (unless (ros::ok) (return-from wander nil))
      (setq goal-cds (send *ri* :state :worldcoords))
      (send *ri* :spin-once))

    (setq diff-pos (norm (send goal-cds :difference-position init-cds)))
    (ros::ros-info "moved: ~A -> ~A (~A)" init-cds goal-cds diff-pos)
    (when (< diff-pos stuck-thresh)
      (ros::ros-info "looks robot is stucked. force rotating")
      (send *ri* :go-pos-unsafe 0 0 90))
    t))

(defun look-on-attention (attention)
  (let ((ps (instance geometry_msgs::PoseStamped :init
                      :pose (send attention :target)))
        cds)
    (send ps :header :stamp (ros::time 0))
    (send ps :header :frame_id (send attention :header :frame_id))
    (setq cds (send *tfl* :transform-pose *base-frame-id* ps))
    (unless cds (return-from look-on-attention nil))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :head :look-at (send cds :worldpos))
    (send *ri* :head-angle-vector (send *pr2* :angle-vector)
          (min (* 1000.0 *look-at-interval*) 3000))
    (setq *last-behavior-time* (ros::time-now))
    ))

(defun attention-cb (msg)
  (when (zerop (length (send msg :type)))
    (ros::ros-error "No attention type detected")
    (return-from attention-cb nil))
  (cond
    ((eq (send msg :level) interactive_behavior_201409::Attention::*LEVEL_NORMAL*)
     (ros::ros-info "normal attention received: ~A" (send msg :type))
     (if (assoc (send msg :type) *pending-attention* :test #'string=)
         (setf (cdr (assoc (send msg :type) *pending-attention* :test #'string=)) msg)
         (push (cons (send msg :type) msg) *pending-attention*)))
    ((eq (send msg :level) interactive_behavior_201409::Attention::*LEVEL_IMPORTANT*)
     (ros::ros-warn "important attention received: ~A" (send msg :type))
     (look-on-attention msg)
     (return-from attention-cb t))
    (t
     (ros::ros-error "Invalid attention level")
     (return-from attention-cb t)))

  ;; choose one attention from normal attentions, then run action with it
  (when (> (send (ros::time- (ros::time-now) *last-behavior-time*) :to-sec)
           *look-at-interval*)
    (let ((next-attention
           (cdr (nth (random (length *pending-attention*)) *pending-attention*))))
      (look-on-attention next-attention)
      (setq *pending-attention*
            (remove-if #'(lambda (x)
                           (string= (car x) (send next-attention :type)))
                       *pending-attention*)))))

(defun timer-cb (event)
  (when (and *look-at-enabled*
             (> (send (ros::time- (ros::time-now) *last-behavior-time*) :to-sec)
                *look-at-interval*))
    (when *enable-base-movement*
      (if (null *last-base-movement*) (setq *last-base-movement* (ros::time-now)))
      (when (> (send (ros::time- (ros::time-now) *last-base-movement*) :to-sec)
               *move-base-interval*)
        (wander)
        (setq *last-base-movement* (ros::time-now))))

    (let ((msg (instance interactive_behavior_201409::Attention :init)))
      (send msg :type "random")
      (send msg :header :frame_id *sensor-frame-id*)
      (send msg :header :stamp (ros::time-now))
      (send msg :level interactive_behavior_201409::Attention::*LEVEL_NORMAL*)
      (send msg :target
            (ros::coords->tf-pose
             (make-coords :pos (float-vector 1000 (* 350 (random-gauss)) (- (* 0.1 (random-gauss)) 0.02)))))
      (look-on-attention msg))
))

(defun enable-behavior ()
  (setq *look-at-enabled* t)
  (ros::subscribe *attention-topic* interactive_behavior_201409::Attention #'attention-cb 1)
  (ros::ros-info "idle behavior enabled"))

(defun disable-behavior ()
  (setq *look-at-enabled* nil)
  (ros::unsubscribe *attention-topic*)
  (ros::ros-info "idle behavior disabled"))

;; main
(warn ";; (main)~%")
(defun main ()
  (pr2-init)

  (ros::advertise-service
   "~enable"  std_srvs::Empty
   #'(lambda (req)
       (enable-behavior)
       (send req :response)))
  (ros::advertise-service
   "~disable" std_srvs::Empty
   #'(lambda (req)
       (disable-behavior)
       (send req :response)))

  (ros::create-timer 1.0 #'timer-cb)
  (enable-behavior)
  (ros::spin))


(main)
(exit)
