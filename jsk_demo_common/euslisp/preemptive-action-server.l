#!/usr/bin/env roseus

(ros::load-ros-manifest "control_msgs")

(defclass preemptive-joint-trajectory-server
  :super propertied-object
  :slots ((action :forward (:worker))
          reference-robot interface-instance
          rgoal))

(defmethod preemptive-joint-trajectory-server
  (:init (name &key (robot) (interface))
                 (setq action (instance ros::simple-action-server :init name
                                        control_msgs::FollowJointTrajectoryAction
                                        :execute-cb `(lambda (s g) (send ,self :execute-cb s g))
                                        :preempt-cb `(lambda (s g) (send ,self :preempt-cb s g))
                                        :accept-cb  `(lambda (s g) (send ,self :accept-cb s g))))
                   (setq reference-robot robot)
                   (setq interface-instance interface)
                   self)
  (:execute-cb (s g)
               (warn ";; execute ~A ~A~%" s g)
               (let ((msg (send s :result))
                     (traj (send g :goal :trajectory))
                     (stamp (send traj :header :stamp))
;;                     (path-tor (send g :goal :path_tolerance))
;;                     (goal-tor (send g :goal :goal_tolerance))
;;                     (gtime-for (send g :goal :goal_time_tolerance))
                     )
                 (when (and reference-robot interface-instance)
                   (cond ((find-method interface-instance :send-trajectory)
                          (send interface-instance :send-trajectory traj))
                         (t
                          (let ((ret (apply-joint_trajectory traj reference-robot)))
                            (send interface-instance :angle-vector-sequence
                                  (car ret) (cadr ret))))
                         )
                   ;; wait-interpolation
                   (ros::rate 100)
                   (let* ((controller-actions (cdr (assoc 'controller-actions (send interface-instance :slots))))
                         (states (mapcar #'(lambda (ac) (send ac :get-state) controller-actions))))
                     (while (and (ros::ok) (null (memq 3 states)))
                       (send *ri* :update-robot-state)
                       (let ((joint-state (send interface-instance :publish-joint-state))
                             (feedback-msg (instance control_msgs::FollowJointTrajectoryActionFeedback :init)))
                         (send feedback-msg :joint_names (send joint-state :name))
                         (send feedback-msg :desired
                               (instance trajectory_msgs::JointTrajectoryPoint :init
                                         :positions (send joint-state :position)
                                         :velocities (send joint-state :velocity)
                                         :accelerations (send joint-state :effort)
                                         :duration (ros::time- (ros::time-now) stamp)))
                         (send s :publish-feedback feedback-msg)
                         )
                       (ros::spin-once)
                       (ros::sleep)
                       (setq states (mapcar #'(lambda (ac) (send ac :get-state) controller-actions)))
                       )
                     ))
                 (send s :set-succeeded msg)))
  (:preempt-cb (s g)
               (warn ";; preempt ~A ~A~%" s g))
  (:accept-cb (s g)
              (warn ";; accept ~A ~A~%" s g))
  (:spin ()
         (ros::rate 100)
         (do-until-key
             (send self :worker)
           (send action :spin-once)
           (ros::sleep)))
  (:spin-once () (send self :worker) (send action :spin-once))
  )

(defun start-node (&optional (trajectory "preemptive_joint_trajectory"))
  (require :pr2-interface  "package://pr2eus/pr2-interface.l")
  (ros::roseus "preemptive_joint_trajectory")
  (pr2-init)

  (cond
    ((listp trajectory)
     (setq *server-list* nil)
     (dolist (traj trajectory)
       (push
        (instance preemptive-joint-trajectory-server
                  :init (ros::resolve-name traj)
                  :robot *pr2* :interface *ri*) *server-list*))
     (ros::ros-info ";; start follow-servers")
     (ros::rate 100)
     (do-until-key
         (dolist (s *server-list*)
           (send s :spin-once))
       (ros::sleep))
     )
    (t
     (setq *server* (instance preemptive-joint-trajectory-server
                              :init (ros::resolve-name trajectory)
                              :robot *pr2* :interface *ri*))
     (ros::ros-info ";; start follow-server")
     (send *server* :spin))
    ))

(start-node)
