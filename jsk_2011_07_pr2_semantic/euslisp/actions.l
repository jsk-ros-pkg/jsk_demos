#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus_openrave/pr2eus-openrave.l")
(ros::roseus-add-msgs "posedetection_msgs")
(ros::roseus-add-msgs "pr2_gripper_sensor_msgs")
(ros::roseus-add-srvs "jsk_pcl_ros")

;(ros::roseus "grasp_cup")


;;
;; for picking cups
;;
(defun grasp (arm)
  (let* ((av (send *pr2* :angle-vector))
		via1 target)
	(send *ri* :stop-grasp arm)
	(setq via1
		  (make-coords :pos (v+ (send *obj* :worldpos) #f(60 120 -50))
					   :rpy #f(-2.1 0 0)))
	(send *pr2* arm :inverse-kinematics via1 :rotation-axis t)
	(send *ri* :angle-vector (send *pr2* :angle-vector))
	(send *ri* :wait-interpolation)
	;;
	(setq target
		  (make-coords :pos (v+ (send *obj* :worldpos) #f(60 90 -50))
					   :rpy #f(-2.1 0 0)))
	(send *pr2* arm :inverse-kinematics target :rotation-axis t)
	(send *ri* :angle-vector (send *pr2* :angle-vector))
	(send *ri* :wait-interpolation)
	;;
	(send *ri* :move-gripper arm 0.0)
	))

(defun pick (arm)
  (let* ((av (send *pr2* :angle-vector))
		 via1 target vav avs)
	(ros::spin-once)
	(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
	(setq via1
		  (make-coords :pos (send (send *obj* :copy-worldcoords) :worldpos)
					   :rpy (float-vector pi/2 pi/2 pi/2)))
	(send via1 :translate #f(20 60 100) :world)
	;;(send *pr2* arm :inverse-kinematics via1 :rotation-axis :x)
	;;(send *pr2* arm :inverse-kinematics via1 :rotation-axis t)
    (setq avs (send *ri* :move-end-coords-plan
					(send via1 :copy-worldcoords)
					:move-target arm :use-torso t))
	;(send *ri* :angle-vector (send *pr2* :angle-vector))
	(send *ri* :wait-interpolation)
	(send *ri* :stop-grasp arm)
	;;
	(ros::spin-once)
	(setq vav (send *ri* :state :potentio-vector))
	(send *pr2* :angle-vector (copy-object vav))
	(setq target
		  (make-coords :pos (send (send *obj* :copy-worldcoords) :worldpos)
					   :rpy (float-vector pi/2 pi/2 pi/2)))
	(send target :translate #f(20 60 -50) :world)
	(send *pr2* :inverse-kinematics target :move-arm arm :rotation-axis t)
	(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
	(send *ri* :wait-interpolation)
	;;
	(send *ri* :move-gripper arm 0.0 :wait t)
	;;
	(send *ri* :angle-vector vav 5000)
	(send *ri* :wait-interpolation)
	;;
	(send *ri* :angle-vector-sequence
		  (reverse (car avs)) (reverse (cadr avs)))
	(send *ri* :wait-interpolation)
	))


(defun hand-over ()
  (let ((client (instance ros::simple-action-client :init "/l_gripper_sensor_controller/event_detector" pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction))
		(goal (instance pr2_gripper_sensor_msgs::PR2GripperEventDetectorActionGoal :init))
		(av (send *ri* :state :potentio-vector))
		(avs (list #f(12.7338 -0.025174 65.3145 70.0513 -93.0477 24.3934
					  -114.929 159.99 -5.00284 74.2661 -104.991 -90.0351
					  69.9982 -5.32433 19.9689 -0.035619 -3.906)
				   #f(12.7313 -0.091677 65.4018 70.0881 -120.296 -72.2013
					  -114.981 159.993 -5.00284 74.2612 -105.001 -90.0019
					  69.9982 -5.32433 20.0038 -0.029619 -3.906))))
  (send *ri* :angle-vector-sequence avs (list 1000 1000))
  (send *ri* :wait-interpolation)
  (send goal :header :stamp (ros::time-now))
  (send goal :goal_id :stamp (ros::time-now))
;;  (send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC*)
  (send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*SLIP*)
  (send goal :goal :command :slip_trigger_magnitude 0.005)
  (send goal :goal :command :acceleration_trigger_magnitude 4.0)
  (send client :send-goal goal)
  (send client :wait-for-result)
  (send *ri* :move-gripper :larm 0.08 :wait t)
  (send *ri* :angle-vector-sequence (append (cdr (reverse avs)) (list av))
		(list 1000 1000))
  (send *ri* :move-gripper :larm 0.00 :wait nil)
  (send *ri* :wait-interpolation)
  ))

(defun objectdetection (msg)
 (let ((obj (car (send msg :objects))) (av (send *pr2* :angle-vector)))
   (ros::ros-info "object detected.")
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
  )

(defun start-subscribe ()
  (ros::subscribe "/ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection))
(defun stop-subscribe ()
  (ros::subscribe "/ObjectDetection" posedetection_msgs::ObjectDetection #'identity))

(defun check (&optional (timeout 10))
  (start-subscribe) (unix::sleep 3)
  (send *obj* :move-to (make-coords) :world)
  (let ((pos1 (send (send *obj* :copy-worldcoords) :worldpos)) pos2 tm)
   (setq tm (ros::time-now))
   (while (ros::ok)
	(ros::spin-once)
    (setq pos2 (send *obj* :worldpos))
	(print (distance pos1 pos2))
	(when (< 0.01 (distance pos1 pos2))
	  (ros::ros-info "ok")
	  (send *irtviewer* :draw-objects)
	  (stop-subscribe)
	  (check-pcl)
	  (return-from check t))
	(when (< timeout (send (ros::time- (ros::time-now) tm) :to-sec))
	  (ros::ros-info "timeout")
	  (stop-subscribe)
	  (return-from check nil))
    (send *viewer* :draw-objects)
    (x::window-main-one)
    (unix::usleep 50000)
   )))

(defun check-pcl ()
 (let ((av (send *pr2* :angle-vector))
	   cam 2dpos req res 3dpos rot pos)
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   ;;
   (setq cam (send *pr2* :narrow_stereo-left))
   (setq 2dpos (send cam :screen-point (send *obj* :worldpos)))
   (let ((minpos (float-vector 1e9 1e9 1e9)) 2dpo)
	 (dotimes (i 5)
	   (dotimes (j 5)
		 (setq 2dpo (v+ 2dpos (float-vector (- (* i 10) 20) (- (* j 10) 20))))
		 (setq req (instance jsk_pcl_ros::TransformScreenpointRequest :init
							 :x (elt 2dpo 0) :y (elt 2dpo 1)))
		 (setq *ray_srv* "/pointcloud_screenpoint/screen_to_point" )
		 (ros::wait-for-service *ray_srv*)
		 (setq res (ros::service-call *ray_srv* req))
		 (setq 3dpos (ros::tf-point->pos (send res :point)))
		 (if (< (norm 3dpos) (norm minpos))
			 (setq minpos 3dpos))
		 ))
	 (setq 3dpos minpos))
   (print (list 2dpos '-> 3dpos))
   (send *obj* :locate (send *pr2* :narrow_stereo_optical_frame :transform-vector 3dpos) :world)
   ;;
   (send *pr2* :angle-vector av)
   (send *irtviewer* :draw-objects)
   ))



;;
;; for wiping table
;;
(defun wipe-table ()
  )