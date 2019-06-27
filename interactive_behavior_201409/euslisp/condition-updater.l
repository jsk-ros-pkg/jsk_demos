#!/usr/bin/env roseus
;; condition-updater.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(provide :condition-updater)

(defvar *world-frame-id* "world")

(require :preemptlib "package://interactive_behavior_201409/euslisp/preemptlib.l")
(require :semantic-world-client "package://semantic_world/src/semantic-world-client.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(ros::load-ros-manifest "pr2_controllers_msgs")
(defclass onhand-updater
  :super condition-updater
  :slots (grasp-thre release-thre larm-state rarm-state))
(defmethod onhand-updater
  (:init (&key (grasp-max-dist 0.005)
               (release-min-dist 0.01))
    (setq grasp-thre grasp-max-dist
          release-thre release-min-dist)
    (send-super :init))
  (:start ()
    (ros::subscribe "l_gripper_controller/state"
                    pr2_controllers_msgs::JointControllerState
                    #'send self :state-callback :larm)
    (ros::subscribe "r_gripper_controller/state"
                    pr2_controllers_msgs::JointControllerState
                    #'send self :state-callback :rarm)
    t)
  (:stop ()
    (ros::unsubscribe "l_gripper_controller/state")
    (ros::unsubscribe "r_gripper_controller/state")
    (setq larm-state nil rarm-state nil)
    t)
  (:wait-for-ready (&optional timeout)
    (when (send *ri* :simulation-modep) (return-from :wait-for-ready nil))
    (ros::rate 1)
    (let ((start-time (ros::time-now)))
      (while (ros::ok)
        (when (and larm-state rarm-state)
          (ros::ros-info "[~A] :larm = ~A, :rarm = ~A" (send self :name) larm-state rarm-state)
          (return-from :wait-for-ready t))
        (when (and timeout
                   (> (send (ros::time- (ros::time-now) start-time) :to-sec) timeout))
          (return-from :wait-for-ready nil))
        (ros::spin-once)
        (ros::sleep))))
  (:state-callback (arm msg)
    (let ((actual (send msg :process_value))
          (desired (send msg :set_point))
          (err (send msg :error)))
      (cond
       ((and (< desired 0.0001)
             (> actual grasp-thre))
        (case arm
          (:larm (setq larm-state :onhand))
          (:rarm (setq rarm-state :onhand))))
       (t
        (case arm
          (:larm (setq larm-state :not-onhand))
          (:rarm (setq rarm-state :not-onhand)))))))
  (:update-init (init goal)
    (let ((ret (copy-object init))
          (last-state (send self :last-state (* 60 60)))
          obj)
      ;; larm
      (cond
       ;; onhand -> on
       ((and (eq larm-state :not-onhand)
             (send self :onhand ret 'larm))
        (setq ret (mapcar #'(lambda (x)
                              (if (and (eq (car x) 'onhand)
                                       (eq (caddr x) 'larm))
                                  `(on ,(cadr x) ,(send self :at init))
                                x)) ret)))
       ;; on -> onhand
       ((and (eq larm-state :onhand)
             (null (send self :onhand ret 'larm)))
        ;; recall onhand object from last state
        (setq obj (send self :onhand last-state 'larm))
        (ros::ros-info "last-onhand=~A" obj)
        (when obj
          (push `(onhand ,(car obj) larm) ret)
          (setq ret (remove `(on ,(car obj) ,(send self :at ret)) ret :test #'equal)))
        ))
      ;; rarm
      (cond
       ;; onhand -> on
       ((and (eq rarm-state :not-onhand)
             (send self :onhand ret 'rarm))
        (setq ret (mapcar #'(lambda (x)
                              (if (and (eq (car x) 'onhand)
                                       (eq (caddr x) 'rarm))
                                  `(on ,(cadr x) ,(send self :at init))
                                x)) ret)))
       ;; on -> onhand
       ((and (eq rarm-state :onhand)
             (null (send self :onhand ret 'rarm)))
        ;; recall onhand object from last state
        (setq obj (send self :onhand last-state 'rarm))
        (ros::ros-info "last-onhand=~A" obj)
        (when obj
          (push `(onhand ,(car obj) rarm) ret)
          (setq ret (remove `(on ,(car obj) ,(send self :at ret)) ret :test #'equal)))
        ))
      ret))
  (:update-goal (init goal) goal)
) ;; onhand-updater
(add-updater onhand-updater)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass at-updater
  :super condition-updater
  :slots (scene))
(defmethod at-updater
  (:init (&optional (scn))
    (setq scene scn)
    (send-super :init))
  (:start ()
    (when (and (null scene) (boundp '*scene*) *scene*)
      (setq scene *scene*))
    (unless scene
      (error "scene is not initialized"))
    (unless (and (boundp '*ri*) *ri*)
      (error "*ri* is not initialized")))
  (:stop () t)
  (:wait-for-ready (&optional timeout) t)
  (:update-init (init goal)
    (let ((spots (get-all-spots scene))
          (robot-pose (send *ri* :state :worldcoords))
          (ret (copy-object init))
          nearest dist)
      (sort spots #'<=
            #'(lambda (s)
                (norm (send robot-pose :difference-position (cdr s)))))
      (setq nearest (car spots)
            dist (norm (send robot-pose :difference-position (cdr nearest))))
      ;; (ros::ros-info "dist to ~A: ~A" (car nearest) dist)
      (unless (< dist 400.0)
        (ros::ros-info "no near spot")
        (return-from :update-init init))
      (ros::ros-info "the robot is near ~A" (car nearest))
      ;; 0. trim compound words
      (setq nearest
          (if (search "-" (car nearest))
              (subseq (car nearest) (1+ (search "-" (car nearest) :from-end t)))
              (car nearest)))
      ;; 1. remove at predicate
      (setq ret (remove-if #'(lambda (x) (eq (car x) 'robot-at)) ret))
      ;; 2. insert at
      (push `(robot-at ,(read-from-string nearest)) ret)
      ret))
  (:update-goal (init goal) goal)
) ;; at-updater
(add-updater at-updater)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(ros::load-ros-manifest "mbtk")
(defclass episodic-object-updater
  :super condition-updater
  :slots (available-labels query-srv-name))
(defmethod episodic-object-updater
  (:init ()
    (setq available-labels (list "cup" "bottle" "bowl" "spoon"))
    (setq query-srv-name "/ssd/query_position")
    (send-super :init))
  (:start ()
    (when (or (not (boundp '*tfl*)) (null *tfl*))
      (setq *tfl* (instance ros::transform-listener :init)))
    t)
  (:stop () t)
  (:wait-for-ready (&optional (timeout 10))
    (unless (ros::wait-for-service query-srv-name timeout)
      (ros::ros-error "[~A] service not advertised: ~A" (send self :name) query-srv-name)
      (return-from :wait-for-ready nil))
    t)
  (:update-init (init goal)
    (let* ((init-objs (send self :objects init))
           (goal-objs (send self :objects goal))
           (new-init (copy-object init))
           (unknown-objs (set-difference goal-objs init-objs))
           res cds-lst sexp)
      (dolist (obj unknown-objs)
        (ros::ros-info "[~A] querying position of ~A" (send self :name) (string-downcase obj))
        (setq res (ros::service-call query-srv-name
                                     (instance mbtk::QueryObjectPositionRequest :init
                                               :class_label (string-downcase obj))))
        (unless (send *tfl* :wait-for-transform *world-frame-id*
                      (send res :object_positions :header :frame_id) (ros::time 0) 10)
          (ros::ros-error "[~A] failed to lookup transform: ~A ~A"
                          (send self :name) *world-frame-id*
                          (send res :object_positions :header :frame_id))
          (return-from :update-init init))
        (setq cds-lst (mapcar #'(lambda (pc)
                                  (let ((ps (instance geometry_msgs::PoseStamped :init)))
                                    (send ps :header (send res :object_positions :header))
                                    (send ps :header :stamp (ros::time 0))
                                    (send ps :pose (send pc :pose))
                                    (send *tfl* :transform-pose *world-frame-id* ps)))
                              (send res :object_positions :poses)))
        (setq sexp (mapcar #'(lambda (cds)
                               (cdr (assoc :states (inspect-item :coords cds :simplify t :single-in t))))
                           cds-lst))
        (when (car sexp)
          (dolist (cnd (car sexp))
            ;; remap
            (cond
             ((eq (car cnd) 'in) (setf (car cnd) 'on))
             ((eq (car cnd) 'located-at) (setf (car cnd) 'robot-at)))
            (setq cnd (replace-symbol 'target obj cnd))
            (ros::ros-info "[~A] ~A" (send self :name) cnd)
            (push cnd new-init)))) ;; dolist obj end
        new-init))
  (:update-goal (init goal) goal)
)
(add-updater episodic-object-updater)

