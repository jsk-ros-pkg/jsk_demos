#!/usr/bin/env roseus
(ros::roseus "move_by_joy")
(ros::roseus-add-msgs "geometry_msgs")
;;(defparameter *result-topic* "/navigation/unsafe_vel")
(defparameter *result-topic* "/teleop/joy_vel")
(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")



(defun move-by-joy ()
  (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
    (send req :config :doubles
	  (list (instance dynamic_reconfigure::DoubleParameter :init
			  :name "inflation_radius" :value 0.1)))
    (ros::service-call "/safe_base_node/local_costmap/set_parameters" req))


  (ros::advertise *result-topic* geometry_msgs::Twist 1)

    (let ((msg (instance geometry_msgs::Twist :init))
          (linear (instance geometry_msgs::Vector3 :init :x 1 :y 0 :z 0))
          (angular (instance geometry_msgs::Vector3 :init :x 0 :y 0 :z 0)))
      (do-until-key
       (send msg :linear linear)
       (send msg :angular angular)
       (ros::publish *result-topic* msg)

       (ros::ros-info "PUBLISH ~A to ~A" msg *result-topic*)
       )

      ;; publish zero if key pressed
      (setq linear (instance geometry_msgs::Vector3 :init :x 0 :y 0 :z 0))
      (setq angular (instance geometry_msgs::Vector3 :init :x 0 :y 0 :z 0))
      (send msg :linear linear)
      (send msg :angular angular)
      (ros::publish *result-topic* msg)
      (ros::ros-info "PUBLISH ~A to ~A" msg *result-topic*)
    )
  )

(move-by-joy)
