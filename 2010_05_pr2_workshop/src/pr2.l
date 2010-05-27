(load (format nil "~A/src/urdf2eus.l"
              (read-line (piped-fork "rospack find eusurdf"))))
(load (format nil "~A/pr2model.l"
              (read-line (piped-fork "rospack find pr2eus"))))

(roseus-add-msgs "roslib")
(roseus-add-msgs "std_msgs")
(roseus-add-msgs "roseus")
(roseus-add-msgs "nav_msgs")
(roseus-add-msgs "geometry_msgs")
(roseus-add-msgs "sensor_msgs")
(roseus-add-msgs "pr2_controllers_msgs")
(roseus-add-actionlibs "pr2_controllers_msgs")

(unless (boundp '*pickview*)
  (pickview))

(defvar *pr2* (pr2))

(defvar *pr2-state* nil
  "*pr2-state* update in ros::spin-onece")

(defun update-pr2-joint-angles (pr2-model pr2-state)
  (let ((joint-names (cdr (assoc :name pr2-state)))
        (positions (cdr (assoc :position pr2-state))))
    (mapcar #'(lambda (n p)
                (let ((j (find (intern (string-upcase n))
                               (send pr2-model :joint-list)
                               :key #'(lambda (x) (send x :name)))))
                  (when j
                    (send j :joint-angle (rad2deg p)))))
            (coerce joint-names cons)
            (coerce positions cons))
    (send pr2-model :angle-vector)))

(defun send-pr2-joint-angles (pr2-model
                              &optional
                              (duration
                               (let ((tm (ros::time))) (send tm :sec 10) tm)))
  (let ((joints
         (send pr2-model :middle-body-joint-angle-list)))
  (let ((joint-names (mapcar #'car joints))
        (positions (mapcar #'deg2rad (mapcar #'cdr joints))))
    (let ((goal (instance pr2_controllers_msgs::JointTrajectoryActionGoal
                          :init)))
      (send goal :header :stamp (ros::time-now))
      (send goal :goal :trajectory :joint_names joint-names)
      (send goal :goal :trajectory :points
            (list (instance trajectory_msgs::JointTrajectoryPoint
                            :init
                            :positions positions
                            :time_from_start
                            duration)))
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
    (warning-message 1 "no waiting ~A~%" name)
    (send as :wait-for-server)
    as))

(defvar *pr2-joint-action*
  (init-action-lib "midbody_controller/joint_trajectory_action"))

;;(objects (list *pr2*))
;;(do-until-key (ros::spin-once) (update-pr2-joint-angles *pr2* *pr2-state*))
