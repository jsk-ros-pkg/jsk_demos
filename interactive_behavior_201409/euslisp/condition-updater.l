#!/usr/bin/env roseus
;; condition-updater.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(provide :condition-updater)

(require :preemptlib "package://interactive_behavior_201409/euslisp/preemptlib.l")


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(ros::load-ros-manifest "pr2_controllers_msgs")
(defclass onhand-updater
  :super condition-updater
  :slots (grasp-thre release-thre larm-state rarm-state))
(defmethod onhand-updater
  (:init (&key (grasp-max-dist 0.01)
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
  (:update-init (st)
    (let ((ret (copy-object st))
          obj)
      ;; larm
      (cond
       ;; onhand -> on
       ((and (eq larm-state :not-onhand)
             (send self :onhand ret 'larm))
        (setq ret (mapcar #'(lambda (x)
                              (if (and (eq (car x) 'onhand)
                                       (eq (caddr x) 'larm))
                                  `(on ,(cadr x) ,(send self :at st))
                                x)) ret)))
       ;; on -> onhand
       ((and (eq larm-state :onhand)
             (null (send self :onhand ret 'larm)))
        ;; unnecessary?
        ))
      ;; rarm
      (cond
       ;; onhand -> on
       ((and (eq rarm-state :not-onhand)
             (send self :onhand ret 'rarm))
        (setq ret (mapcar #'(lambda (x)
                              (if (and (eq (car x) 'onhand)
                                       (eq (caddr x) 'rarm))
                                  `(on ,(cadr x) ,(send self :at st))
                                x)) ret)))
       ;; on -> onhand
       ((and (eq rarm-state :onhand)
             (null (send self :onhand ret 'rarm)))
        ;; unnecessary?
        ))
      ret))
  (:update-goal (st) st)
) ;; onhand-updater
(add-updater onhand-updater)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass at-updater
  :super condition-updater
  :slots (scene))
(defmethod at-updater
  (:init (&optional (scn))
    (if scn
        (setq scene scn)
      (if (and (boundp '*scene*) *scene*)
          (setq scene *scene*)
        (error "no scene")))
    (unless (and (boundp '*ri*) *ri*)
      (error "*ri* is not initialized"))
    (send-super :init))
  (:start () t)
  (:stop () t)
  (:wait-for-ready (&optional timeout) t)
  (:update-init (st)
    (let ((spots (remove-if-not #'(lambda (x)
                                    (substringp "-front" x))
                                (send scene :spots)))
          (robot-pose (send *ri* :state :worldcoords))
          nearest spot)
      (sort spots #'<=
            #'(lambda (s)
                (norm (send robot-pose :difference-position s))))
      (setq nearest (car spots))
      (ros::ros-info "dist to ~A: ~A" (send nearest :name)
                     (norm (send robot-pose :difference-position nearest)))
      (if (< (norm (send robot-pose :difference-position nearest))
             1000.0)
          ;; TODO
          )

    (error "Not Implemented"))
  (:update-goal (st)
    (error "Not Implemented"))
) ;; at-updater
(add-updater at-updater)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass episodic-object-updater
  :super condition-updater
  :slots ())
(defmethod episodic-object-updater
  (:init ())
  (:start ()
    (error "Not implemented"))
  (:stop ()
    (error "Not implemented"))
  (:wait-for-ready (&optional timeout)
    (error "Not implemented"))
  (:update-init (st)
    (error "Not Implemented"))
  (:update-goal (st)
    (error "Not Implemented")))
(add-updater episodic-object-updater)
