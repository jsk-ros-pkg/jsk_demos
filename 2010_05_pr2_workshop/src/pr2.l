(load (format nil "~A/euslisp/roseus.l"
              (read-line (piped-fork "rospack find roseus"))))
(load (format nil "~A/src/urdf2eus.l"
              (read-line (piped-fork "rospack find eusurdf"))))
(load (format nil "~A/pr2model.l"
              (read-line (piped-fork "rospack find pr2eus"))))
(load (format nil "~A/euslisp/actionlib.l"
              (read-line (piped-fork "rospack find roseus"))))

(roseus-add-msgs "roslib")
(roseus-add-msgs "std_msgs")
(roseus-add-msgs "roseus")
(roseus-add-msgs "nav_msgs")
(roseus-add-msgs "geometry_msgs")
(roseus-add-msgs "sensor_msgs")
(roseus-add-msgs "pr2_controllers_msgs")
(roseus-add-msgs "pr2_msgs")
(roseus-add-actionlibs "pr2_controllers_msgs")

(unless (boundp '*pickview*)
  (pickview))

(defvar *pr2* (pr2))
(objects (list *pr2*))

(defvar *pr2-state* nil
  "*pr2-state* update in ros::spin-onece")

;;
;; for send joint angles to pr2
;;
(defun update-pr2-joint-angles (pr2-model pr2-state)
  (let ((joint-names (cdr (assoc :name pr2-state)))
        (positions (cdr (assoc :position pr2-state))))
    (mapcar #'(lambda (n p)
                (let ((j (find (intern (string-upcase n))
                               (send pr2-model :joint-list)
                               :key #'(lambda (x) (send x :name)))))
                  (when j
		    (if (derivedp j rotational-joint)
			(send j :joint-angle (rad2deg p))
		      (send j :joint-angle (* 1000 p))))))
            (coerce joint-names cons)
            (coerce positions cons))
    (send pr2-model :angle-vector)))

(defun send-pr2-joint-angles (pr2-model
                              &optional
                              (duration 10))
  (let ((joints (send pr2-model :full-body-joint-angle-list))
        (tm (ros::time duration)))
    (let ((joint-names (mapcar #'(lambda (x)
                                   (string-downcase (string x)))
                               (send-all joints :name)))
          (positions ;;(mapcar #'deg2rad (mapcar #'cdr joints))))
           (mapcar #'(lambda (x)
                       (if (derivedp x rotational-joint)
                           (deg2rad (send x :joint-angle))
                         (* 0.001 (send x :joint-angle))))
                   joints)))
      (print joint-names)
      (let ((goal (instance pr2_controllers_msgs::JointTrajectoryActionGoal
                            :init)))
        (send goal :header :seq 1)
        (send goal :header :stamp (ros::time-now))
        (send goal :goal :trajectory :joint_names joint-names)
        (send goal :goal :trajectory :header :stamp (ros::time-now))
        (send goal :goal :trajectory :points
              (list (instance trajectory_msgs::JointTrajectoryPoint
                              :init
                              :positions positions
                              :velocities (make-list (length positions) :initial-element 0)
                              :time_from_start
                              tm)))
        (send *pr2-joint-action* :send-goal goal)))))

(defun pr2-state-callback (msg)
  ;; not cool
  (setq *pr2-state* (list (cons :name (send msg :name))
                          (cons :position (send msg :position)))))

;; add subscribre
(ros::subscribe "/joint_states" sensor_msgs::JointState
  #'pr2-state-callback)

;; initialize actionlib client
(defun init-action-lib (name)
  (let ((as (instance ros::simple-action-client :init
                      name pr2_controllers_msgs::JointTrajectoryAction)))
    (warning-message 1 "waiting ~A~%" name)
    (send as :wait-for-server)
    as))

(defvar *pr2-joint-action*
  (init-action-lib "fullbody_controller/joint_trajectory_action"))

;;
;; for gripper command
;;
(setq *r-gripper-client* (instance ros::simple-action-client :init
                                 "/r_gripper_controller/gripper_action"
                                 pr2_controllers_msgs::Pr2GripperCommandAction))
(setq *l-gripper-client* (instance ros::simple-action-client :init
                                 "/l_gripper_controller/gripper_action"
                                 pr2_controllers_msgs::Pr2GripperCommandAction))
(send *r-gripper-client* :wait-for-server)
(send *l-gripper-client* :wait-for-server)

(defun move-gripper (arm pos &key (effort -1) (wait t))
  (let* ((goal (instance pr2_controllers_msgs::Pr2GripperCommandActionGoal :init))
         (c (send (send goal :goal) :command))
	 (client (if (eq arm :rarm) *r-gripper-client* *l-gripper-client*)))
    (send c :position pos)
    (send c :max_effort effort)
    (send client :send-goal goal)
    (ros::spin-once)
    (if wait (send client :wait-for-result))
    ))

;;
;; for reading fingertip pressure
;;
(defun pressure-cb (arm msg)
  (if (boundp '*p*) (setq *pre-p* *p*))
  (setq *p* (cons (send msg :l_finger_tip) (send msg :r_finger_tip)) )
  ;;(pprint *p*)
  (when (and (boundp '*pre-p*) *pre-p*)
    (setq *diff-p* (cons (v- (car *p*) (car *pre-p*))
                         (v- (cdr *p*) (cdr *pre-p*))))
    ;;(pprint *diff-p*)
    )
  )
(setq *r-pressure-sub* (ros::subscribe "pressure/r_gripper_motor"
                                     pr2_msgs::PressureState
				     #'(lambda(msg)(pressure-cb :rarm msg))))
(setq *l-pressure-sub* (ros::subscribe "pressure/l_gripper_motor"
                                     pr2_msgs::PressureState
				     #'(lambda(msg)(pressure-cb :larm msg))))
