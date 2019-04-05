(ros::load-ros-manifest "control_msgs")

(defun init ()
  (ros::load-ros-manifest "roseus")
  (ros::roseus "juggle-system")

  ;;
  ;; init variables
  ;;
  (setq *idle-diabolo-pitch* 0)
  (setq *idle-diabolo-yaw* 0)
  
  (setq *diabolo-system-input-arm* 0)
  (setq *diabolo-system-input-base* 0)
  
  ;;
  ;; Subscriber
  ;;
  ;; subscribe idle diabolo state
  (ros::create-nodehandle "idle")
  (ros::subscribe "calc_idle_diabolo_state/diabolo_state" std_msgs::Float64MultiArray
  		  #'(lambda (msg) (progn
  				    (setq *idle-diabolo-pitch* (elt (send msg :data) 0))
  				    (setq *idle-diabolo-yaw* (elt (send msg :data) 1))
  				    ))
		  :groupname "idle")
  (ros::subscribe "joy" sensor_msgs::Joy
  		  #'(lambda (msg) (progn
                          (setq *stop-feedback* (elt (send msg :buttons) 14))
                          ))
        :groupname "idle")

  
  ;; subscribe joy
  (ros::create-nodehandle "joy")
  ;; (ros::subscribe "joy" sensor_msgs::Joy
  ;; 		  #'(lambda (msg) (setq *joy* (send msg :axes)))
	;; 	  :groupname "joy")

  ;; subscribe DiaboloSystem
  (ros::create-nodehandle "diabolo-system")
  (ros::subscribe "diabolo_system/diabolo_input" std_msgs::Float64MultiArray
  		  #'(lambda (msg) (progn
				    (setq *diabolo-system-input-arm* (elt (send msg :data) 0))
				    (setq *diabolo-system-input-base* (elt (send msg :data) 1))
				    ))
		  :groupname "diabolo-system")
  (ros::create-nodehandle "diabolo-system-simulate")
  (ros::subscribe "diabolo_system/diabolo_state" std_msgs::Float64MultiArray
  		  #'(lambda (msg) (progn
				    (setq *idle-diabolo-pitch* (elt (send msg :data) 0))
				    (setq *idle-diabolo-yaw* (elt (send msg :data) 1))
				    ))
		  :groupname "diabolo-system-simulate")

  ;;
  ;; Publisher
  ;;
  ;; publish whether idle or not
  (ros::advertise "idle" std_msgs::float64 1)

  ;; publish larm
  (ros::advertise "/l_arm_controller/follow_joint_trajectory/goal" control_msgs::FollowJointTrajectoryActionGoal 1)  

  ;;
  ;; init PR2
  ;;
  (load "package://pr2eus/pr2-interface.l")
  (pr2-init)
  
  (send *ri* :angle-vector (send *pr2* :reset-pose) 5000)
  (setq *ac* (pr2-interface-move-base-trajectory-action *ri*))
  
  (send *ri* :start-grasp)
  (send *pr2* :head-neck-p :joint-angle 50)

  (solve-ik)
  
  (objects (list *pr2*))
  
  (load "package://diabolo_pr2_201806/euslisp/utils.l")
  (load "package://diabolo_pr2_201806/euslisp/idle.l")
  )

(defun solve-ik ()
  (send *pr2* :reset-pose)
  (send *pr2* :start-grasp)  
  (send *pr2* :head-neck-p :joint-angle 50)
  
  (progn ;; default 500~900
    (setq *default-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -120 1050)))
    (send *pr2* :rarm :inverse-kinematics *default-right-cube*)  
    (setq *default-left-cube* (make-cube 100 100 100 :pos (float-vector 700 120 1050)))
    (send *pr2* :larm :inverse-kinematics *default-left-cube*)   
    (setq *default-pos* (send *pr2* :angle-vector))
    )

  (progn  ;; idle
    (setq *idle-right-up-cube* (make-cube 100 100 100 :pos (float-vector 700 -150 1150)))
    (send *pr2* :rarm :inverse-kinematics *idle-right-up-cube*)  
    (setq *idle-left-down-cube* (make-cube 100 100 100 :pos (float-vector 700 150 950)))
    (send *pr2* :larm :inverse-kinematics *idle-left-down-cube*)    
    (setq *idle-1-pos* (send *pr2* :angle-vector))
    
    (setq *idle-right-down-cube* (make-cube 100 100 100 :pos (float-vector 700 -150 950)))
    (send *pr2* :rarm :inverse-kinematics *idle-right-down-cube*)
    (setq *idle-left-up-cube* (make-cube 100 100 100 :pos (float-vector 700 150 1150)))
    (send *pr2* :larm :inverse-kinematics *idle-left-up-cube*)   
    (setq *idle-2-pos* (send *pr2* :angle-vector))
    )
  
  (progn ;; roll
    (setq *roll-1-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -50 1050)))
    (send *pr2* :rarm :inverse-kinematics *roll-1-right-cube*)
    (setq *roll-1-left-cube* (make-cube 100 100 100 :pos (float-vector 700 250 1050)))
    (send *pr2* :larm :inverse-kinematics *roll-1-left-cube*)   
    (setq *roll-1-pos* (send *pr2* :angle-vector))

    (setq *roll-2-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -50 900)))
    (send *pr2* :rarm :inverse-kinematics *roll-2-right-cube*)  
    (setq *roll-2-left-cube* (make-cube 100 100 100 :pos (float-vector 700 250 900)))
    (send *pr2* :larm :inverse-kinematics *roll-2-left-cube*)    
    (setq *roll-2-pos* (send *pr2* :angle-vector))
    
    (setq *roll-3-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -700 900)))
    (send *pr2* :rarm :inverse-kinematics *roll-3-right-cube*)  
    (setq *roll-3-left-cube* (make-cube 100 100 100 :pos (float-vector 700 -400 900)))
    (send *pr2* :larm :inverse-kinematics *roll-3-left-cube* :rotation-axis :z)    
    (setq *roll-3-pos* (send *pr2* :angle-vector))
    
    (setq *roll-4-right-cube* (make-cube 100 100 100 :pos (float-vector 700 -150 850)))
    (send *pr2* :rarm :inverse-kinematics *roll-4-right-cube*)  
    (setq *roll-4-left-cube* (make-cube 100 100 100 :pos (float-vector 700 150 850)))
    (send *pr2* :larm :inverse-kinematics *roll-4-left-cube*)    
    (setq *roll-4-pos* (send *pr2* :angle-vector))
    )
  )
