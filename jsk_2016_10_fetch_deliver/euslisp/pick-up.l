#!/usr/bin/env roseus

(load "package://fetcheus/fetch-interface.l")
(fetch-init)

(ros::load-ros-manifest "roseus")

(ros::roseus "pick_up")
(ros::advertise "received" std_msgs::string 1)

(defun standby
  ()
  (let ()
    (send *ri* :start-grasp)
    (send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(800 0 1000) :rpy (float-vector 0 (/ pi -3) 0)))
    (send *fetch* :rarm :wrist-r :joint-angle -45)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 6000)
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp)

    ))

(defun try-grasp
  (&key effort)
  (let (result)
    (send *ri* :speak "Please place the package in my hand." :wait t)
    (send *ri* :ros-wait 1)
    (send *ri* :speak "Grasping in three" :wait t)
    (send *ri* :ros-wait 1)
    (send *ri* :speak "two" :wait t)
    (send *ri* :ros-wait 1)
    (send *ri* :speak "one" :wait t)
    (send *ri* :ros-wait 1)
    (setq result (send *ri* :start-grasp :effort effort))
    (send *ri* :wait-interpolation)
    (send *ri* :ros-wait 1)
    
    (send result :position))
  )

(defun go-back
  ()
  (send *ri* :angle-vector (send *fetch* :angle-vector #f(9.14772 75.5041 86.975 -11.4777 90.0 0.038312 90.0 -0.038098 0.407858 0.540208)) 6000)
  (send *ri* :wait-interpolation)

  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "ok"))
  (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "received" msg))

(defun pickup
  (&key (times 3))
  (let ((effort 50)
	(max-effort 80)
	(count 0)
	result)
    (standby)
    (setq result (try-grasp :effort effort))
    (while (and (< result 0.01) (< count times))
      (incf count)
      (if (< effort max-effort)
	  (setq effort (+ 10 effort)))
      (send *ri* :stop-grasp)
      (setq result (try-grasp :effort effort)))	  
    (go-back)
    ))

;;callback function
(defun pick-up-cb (msg)
  (let (flag)
    (setq flag (send msg :data))
    (when (string= flag "go")
	(pickup)
	(format t "Received package"))
    ))

(ros::subscribe "pick_up" std_msgs::string #'pick-up-cb)


(ros::spin)





