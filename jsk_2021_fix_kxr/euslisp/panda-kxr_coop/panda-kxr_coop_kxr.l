#!/usr/bin/env roseus

(load "/home/amabe/prog/rcb4eus/rosrcb4.l")
(make-kxr-robot "kxrl2l5a3h2g")
(send *ri* :ros-open)
(send *ri* :timer-on)
(send *ri* :hold-all)
(init-publish)

(ros::roseus "kxr_robot")

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::load-ros-manifest "std_msgs")
(ros::load-ros-manifest "sensor_msgs")

(defun servo-onoff()
  (setq msg (send (one-shot-subscribe "/panda_order" std_msgs::String) :data))
  (print msg)
  (cond ((string-equal msg "rarm-free")
	 (progn
	   (send *ri* :rarm-free)))
	((string-equal msg "larm-free")
	 (progn
	   (send *ri* :larm-free)))
	((string-equal msg "rarm-hold")
	 (progn
	   (send *ri* :rarm-hold)))
	((string-equal msg "larm-hold")
	 (progn
	   (send *ri* :larm-hold)))))

(defun take-posture()
  (setq msg (send (one-shot-subscribe "/panda_order" std_msgs::String) :data))
  (cond ((string-equal msg "RARM-SHOULDER-R")
	 (progn
	   (send *robot* :angle-vector #f(0.0 -10.0 40.0 -20.0 0.0 0.0 -10.0 40.0 -20.0 0.0 -90.0 -90.0 0.0 -27.0 -27.0 -90.0 90.0 0.0 -27.0 -27.0 0.0 10.0))
	   (send *ri* :angle-vector (send *robot* :angle-vector) 200)))
	((string-equal msg "RARM-ELBOW-P")
	 (progn
	   (send *robot* :angle-vector #f(0.0 -10.0 40.0 -20.0 0.0 0.0 -10.0 40.0 -20.0 0.0 0.0 -90.0 0.0 -27.0 -27.0 0.0 90.0 0.0 -27.0 -27.0 0.0 10.0))
	   (send *ri* :angle-vector (send *robot* :angle-vector) 200)))))

(defun check-move()
  (setq msg (send (one-shot-subscribe "/panda_order" std_msgs::String) :data))
  (cond ((string-equal msg "check-rarm-elbow-p")
	 (progn
	   (send *robot* :rarm-elbow-p :joint-angle 40)
	   (send *ri* :angle-vector (send *robot* :angle-vector) 150)
	   (unix:sleep 3)
	   
	   (send *robot* :rarm-elbow-p :joint-angle -40)
	   (send *ri* :angle-vector (send *robot* :angle-vector) 150)
	   (unix:sleep 3)
	   
	   (send *robot* :rarm-elbow-p :joint-angle 0)
	   (send *ri* :angle-vector (send *robot* :angle-vector) 150)
	   (unix:sleep 3)))))

(joint-publish)

(take-posture)

(servo-onoff)

(setq f 0)
(while (= f 0)
  (print "waiting for panda_order")
  (setq msg (send (one-shot-subscribe "/panda_order" std_msgs::String) :data))
  (print "got msg")
  (print msg)
  (unix:sleep 3)
  (cond  ((string-equal msg "ope-finish")
	  (joint-publish))
	 
	 ((string-equal msg "check-rarm")
	  (progn
	    (send *ri* :rarm-hold)
	    (send *robot* :rarm-elbow-p :joint-angle 40)
	    (send *ri* :angle-vector (send *robot* :angle-vector) 150)
	    (unix:sleep 2)
	    
	    (send *robot* :rarm-elbow-p :joint-angle -40)
	    (send *ri* :angle-vector (send *robot* :angle-vector) 150)
	    (unix:sleep 2)
	    
	    (send *robot* :rarm-elbow-p :joint-angle 0)
	    (send *ri* :angle-vector (send *robot* :angle-vector) 150)
	    (unix:sleep 2)
	    (print "rarmmove")))  
	 ((string-equal msg "ope-success")
	  (setq f 1)))
  (unix:sleep 3))

