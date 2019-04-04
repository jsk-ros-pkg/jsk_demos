(defun publish-left-angle-vector (time)   ;; after solving ik
  (setq *idle-angle-vector-joint-names* (mapcar #'(lambda (val) (send (send val :joint) :name)) (send *pr2* :larm)))  
  (setq *idle-angle-vector-positions* (map float-vector #'deg2rad (send *pr2* :larm :angle-vector)))
  (setq *idle-angle-vector-time* time)

  ;; init msg
  (setq *idle-angle-vector-msg* (instance control_msgs::FollowJointTrajectoryActionGoal :init))

  ;; assign data to msg
  (setq *ros-time-sec* (send (send (ros::time) :now) :sec))
  (setq *ros-time-nsec* (send (send (ros::time) :now) :nsec))  
  (let ((i (instance trajectory_msgs::JointTrajectoryPoint :init)))
    (send i :positions  *idle-angle-vector-positions*)
    (send i :velocities  #f(0.0 0.0 0.0 0.0 0.0 0.0 0.0))
    (send (send i :time_from_start) :sec (/ *idle-angle-vector-time* 1000))
    (send (send i :time_from_start) :nsec (- *idle-angle-vector-time* (* (/ *idle-angle-vector-time* 1000) 1000)))
    (send (send (send (send *idle-angle-vector-msg* :goal :trajectory) :header) :stamp) :sec *ros-time-sec*)
    (send (send (send (send *idle-angle-vector-msg* :goal :trajectory) :header) :stamp) :nsec *ros-time-nsec*)
    (send *idle-angle-vector-msg* :goal :trajectory :points (list i))
    )

  (send *idle-angle-vector-msg* :header :seq 1)
  (send (send (send *idle-angle-vector-msg* :header) :stamp)
	:sec *ros-time-sec*)
  (send (send (send *idle-angle-vector-msg* :header) :stamp)
	:nsec *ros-time-nsec*)

  (send *idle-angle-vector-msg*
	:goal :trajectory :joint_names *idle-angle-vector-joint-names*)

  (ros::publish "/l_arm_controller/follow_joint_trajectory/goal" *idle-angle-vector-msg*)
  )

(defun publish-right-angle-vector (time)   ;; after solving ik
  (setq *idle-angle-vector-joint-names* (mapcar #'(lambda (val) (send (send val :joint) :name)) (send *pr2* :rarm)))  
  (setq *idle-angle-vector-positions* (map float-vector #'deg2rad (send *pr2* :rarm :angle-vector)))
  (setq *idle-angle-vector-time* time)

  ;; init msg
  (setq *idle-angle-vector-msg* (instance control_msgs::FollowJointTrajectoryActionGoal :init))

  ;; assign data to msg
  (setq *ros-time-sec* (send (send (ros::time) :now) :sec))
  (setq *ros-time-nsec* (send (send (ros::time) :now) :nsec))  
  (let ((i (instance trajectory_msgs::JointTrajectoryPoint :init)))
    (send i :positions  *idle-angle-vector-positions*)
    (send i :velocities  #f(0.0 0.0 0.0 0.0 0.0 0.0 0.0))
    (send (send i :time_from_start) :sec (/ *idle-angle-vector-time* 1000))
    (send (send i :time_from_start) :nsec (- *idle-angle-vector-time* (* (/ *idle-angle-vector-time* 1000) 1000)))
    (send (send (send (send *idle-angle-vector-msg* :goal :trajectory) :header) :stamp) :sec *ros-time-sec*)
    (send (send (send (send *idle-angle-vector-msg* :goal :trajectory) :header) :stamp) :nsec *ros-time-nsec*)
    (send *idle-angle-vector-msg* :goal :trajectory :points (list i))
    )

  (send *idle-angle-vector-msg* :header :seq 1)
  (send (send (send *idle-angle-vector-msg* :header) :stamp)
	:sec *ros-time-sec*)
  (send (send (send *idle-angle-vector-msg* :header) :stamp)
	:nsec *ros-time-nsec*)

  (send *idle-angle-vector-msg*
	:goal :trajectory :joint_names *idle-angle-vector-joint-names*)

  (ros::publish "/r_arm_controller/follow_joint_trajectory/goal" *idle-angle-vector-msg*)
  )
