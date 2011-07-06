#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(ros::roseus-add-msgs "posedetection_msgs")

;(ros::roseus "grasp_cup")

(defun grasp ()
  (let* ((av (send *pr2* :angle-vector))
		rarm via1 target)
	(send *ri* :stop-grasp :larm)
	(setq via1
		  (make-coords :pos (v+ (send *obj* :worldpos) #f(60 120 -50))
					   :rpy #f(-2.1 0 0)))
	(send *pr2* :larm :inverse-kinematics via1 :rotation-axis t)
	(send *ri* :angle-vector (send *pr2* :angle-vector))
	(send *ri* :wait-interpolation)
	;;
	(setq target
		  (make-coords :pos (v+ (send *obj* :worldpos) #f(60 90 -50))
					   :rpy #f(-2.1 0 0)))
	(send *pr2* :larm :inverse-kinematics target :rotation-axis t)
	(send *ri* :angle-vector (send *pr2* :angle-vector))
	(send *ri* :wait-interpolation)
	;;
	(send *ri* :move-gripper :larm 0.0)
	))

(defun pick ()
  (let* ((av (send *pr2* :angle-vector))
		rarm via1 target)
	(send *ri* :stop-grasp :larm)
	(setq via1
		  (make-coords :pos (v+ (send *obj* :worldpos) #f(40 50 100))
					   :rpy (float-vector pi/2 pi/2 pi/2)))
	(send *pr2* :larm :inverse-kinematics via1 :rotation-axis t)
	(send *ri* :angle-vector (send *pr2* :angle-vector))
	(send *ri* :wait-interpolation)
	;;
	(setq target
		  (make-coords :pos (v+ (send *obj* :worldpos) #f(40 80 -40))
					   :rpy (float-vector pi/2 pi/2 pi/2)))
	(send *pr2* :larm :inverse-kinematics target :rotation-axis t)
	(send *ri* :angle-vector (send *pr2* :angle-vector))
	(send *ri* :wait-interpolation)
	;;
	(send *ri* :move-gripper :larm 0.0)
	))


(defun ready ()
  (send *pr2* :larm :angle-vector
		#f(82.8881 1.48526 74.1123 -58.0438 4.72866 -114.589 -104.313))
  (send *ri* :angle-vector (send *pr2* :angle-vector)))

(defun objectdetection (msg)
 (let ((obj (car (send msg :objects))) (av (send *pr2* :angle-vector)))
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (send *obj* :move-to (send *pr2* :narrow_stereo_optical_frame :copy-worldcoords) :world)
   (send *obj* :move-to (ros::tf-pose->coords (send obj :pose)) :local)
   (send *pr2* :angle-vector av)
   ))

(defun init2 ()
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
  (setq *obj* (body+ (make-cylinder 50 90)
					 (send (make-cube 70 10 80) :translate #f(45 0 45))))
  (send *obj* :translate-vertices (float-vector 50 0 0))
  (send *obj* :rotate-vertices pi/2 :y)
  (send *obj* :rotate-vertices (/ pi -4) :x)
  (objects (list *pr2* *obj*))
  (ros::subscribe "/ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection)
  )

(defun check ()
  (do-until-key
   (ros::spin-once)
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (send *viewer* :draw-objects)
   (unix::usleep 50000)
   ))
