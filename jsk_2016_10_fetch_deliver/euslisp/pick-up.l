#!/usr/bin/env roseus

#|
Needs proximity sensor active
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=9600
|#


(load "package://fetcheus/fetch-interface.l")
(fetch-init)

(ros::load-ros-manifest "roseus")
(ros::load-ros-package "force_proximity_ros")

(ros::roseus "proximity_sensor")
(ros::advertise "go_to_room" std_msgs::string 1)

(setq trigger t)

(defun standby
    ()
  (let ()
    (send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(800 0 1000) :rpy (float-vector 0 (/ pi -3) 0)))
    (send *fetch* :rarm :wrist-r :joint-angle -45)
    (send *ri* :start-grasp)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 6000)
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp)
    (send *ri* :speak "Please place the package in my hand." :wait t)
    ))

(defun try-grasp
  (&key effort)
  (let (result)
    (send *ri* :speak "Grasping in three. Two.   One." :wait t)
    (setq result (send *ri* :start-grasp :effort effort))
    (send *ri* :wait-interpolation)
    (send *ri* :ros-wait 0.5)
    
    (send result :position))
  )

(defun go-back
  ()
  (send *ri* :angle-vector (send *fetch* :angle-vector #f(9.14772 75.5041 86.975 -11.4777 90.0 0.038312 90.0 -0.038098 0.407858 0.540208)) 6000)
  (send *ri* :wait-interpolation))

(defun print-ok
  ()  
  (setq trigger nil)
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "home-point"))
  (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "go_to_room" msg))

(defun pickup
  (&key (times 3))
  (let ((effort 50)
	(max-effort 80)
	(count 0)
	result)
    (setq result (try-grasp :effort effort))
    (while (and (< result 0.01) (< count times))
      (incf count)
      (if (< effort max-effort)
	  (setq effort (+ 10 effort)))
      (send *ri* :stop-grasp)
      (setq result (try-grasp :effort effort)))	  
    (go-back)
    (send *ri* :wait-interpolation)
    (print-ok)
    ))


;;callback function
(defun sensor-cb (msg)
  (let ((prox) (max 10000))
    (setq prox (send msg :proximity))
    (print (format t "in callback; trigger: ~a~%" trigger))
    (if (and (> prox max) trigger)
	(progn
	  (print prox)
	  (pickup)))
    ))

(defun pick-up-cb (msg)
  (let (flag)
    (setq flag (send msg :data))
    (when (string= flag "go")
      (print "Running standby")
      (standby)
      (print "Running subscribe callback")
      (ros::subscribe "proximity_sensor" force_proximity_ros::proximity #'sensor-cb)

      )))


(ros::subscribe "pick_up" std_msgs::String #'pick-up-cb)

(ros::spin)





