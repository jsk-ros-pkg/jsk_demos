#!/usr/bin/env roseus

(load "/home/amabe/prog/rcb4eus/rosrcb4.l")
(make-kxr-robot "kxrl2l5a3h2g")
(send *ri* :ros-open)
(send *ri* :timer-on)
(send *ri* :hold-all)
(init-publish)

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::load-ros-manifest "std_msgs")
(ros::load-ros-manifest "sensor_msgs")

(ros::rate 1)

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
	   (send *ri* :angle-vector (send *robot* :angle-vector) 200)
	 ((string-equal msg "RARM-ELBOW-P")
	  (progn
	    (send *robot* :angle-vector #f(0.0 -10.0 40.0 -20.0 0.0 0.0 -10.0 40.0 -20.0 0.0 0.0 -90.0 0.0 -27.0 -27.0 0.0 90.0 0.0 -27.0 -27.0 0.0 10.0))
	    (send *ri* :angle-vector (send *robot* :angle-vector) 200)))))))

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


(take-posture)

;;指令を受けた後腕のサーボを切る
(servo-onoff)

(setq f 0)
(while (= f 0)
  (joint-publish)
  (if ()
      (progn
	(send *ri* :rarm-hold)
	(send *robot* :rarm-elbow-p :joint-angle 40)
	(send *ri* :angle-vector (send *robot* :angle-vector) 150)
	(unix:sleep 3)
	
	(send *robot* :rarm-elbow-p :joint-angle -40)
	(send *ri* :angle-vector (send *robot* :angle-vector) 150)
	(unix:sleep 3)
	
	(send *robot* :rarm-elbow-p :joint-angle 0)
	(send *ri* :angle-vector (send *robot* :angle-vector) 150)
	(unix:sleep 3)

	(if ()
	     (setq f 1))
	)))
 

  
  (unix:sleep 1))

(while (ros::ok)
  (print "hoge")
  (ros::sleep))

;;ケーブルがさせた後腕のサーボon
(servo-onoff)

;;サーボが生き返ったかの確認
(check-move)
