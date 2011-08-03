#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus_openrave/pr2eus-openrave.l")
(ros::roseus-add-msgs "posedetection_msgs")
(ros::roseus-add-msgs "pr2_gripper_sensor_msgs")
(ros::roseus-add-srvs "jsk_pcl_ros")
(ros::roseus-add-msgs "sound_play")

;(ros::roseus "grasp_cup")

;; speak function
(defun speak-jp (jp-str)
  (ros::publish "robotsound"
                (instance sound_play::SoundRequest :init
                          :sound sound_play::SoundRequest::*say*
                          :command sound_play::SoundRequest::*play_once*
                          :arg jp-str)))


(defun pick (arm)
  (let* (tmp via1 target av vav avs)
    (ros::spin-once)
    (setq av (send *ri* :state :potentio-vector))
    (send *pr2* :angle-vector av)
    ;; calculate via point
    ;; tmp : object coords wrt robot coordinates
    ;; *pr2* :world, *obj* :world
    (setq tmp (make-coords :pos (send *pr2* :inverse-transform-vector (send *obj* :worldpos)))) ;; local
    (send tmp :translate #f(40 60 -50)) ;; move-to handle
    (send tmp :newcoords 
	  (rpy-matrix pi/2 pi/2 pi/2) ;; grasp from top
	  ;;(rpy-matrix -pi/2 0 0)        ;; grasp from side
	  (send (send tmp :copy-worldcoords) :worldpos))
    (setq via1 (send tmp :copy-worldcoords))
    (send via1 :translate #f(0 0 150) :world) ;; grasp from top
    ;;(send via1 :translate #f(0 140 100) :world) ;; grasp from side
    (setq target (send tmp :copy-worldcoords))
    (ros::ros-warn (format nil "tmp ~a" tmp))
    (ros::ros-warn (format nil "via1 ~a" via1))
    (setq avs (send *ri* :move-end-coords-plan via1
		    :move-target arm :use-torso t))
	(if (null avs) (return-from pick nil))
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper arm 0.08 :wait t)
    ;; calculate grasp point
    (ros::spin-once)
    (setq vav (send *ri* :state :potentio-vector))
    (send *pr2* :angle-vector (copy-object vav))
    ;;
    (send target :transform *pr2* :parent)
    (print target)
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
	  (append (cdr (reverse (car avs))) (list av)) (reverse (cadr avs)))
    (send *ri* :wait-interpolation)
    t ))


(defun hand-over () ;;  only larm
  (ros::spin-once)
  (let ((client (instance ros::simple-action-client :init "/l_gripper_sensor_controller/event_detector" pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction))
	(goal (instance pr2_gripper_sensor_msgs::PR2GripperEventDetectorActionGoal :init))
	(av (send *ri* :state :potentio-vector))
	(avs (list #f(12.7338 -0.025174 65.3145 70.0513 -93.0477 24.3934
			      -114.929 159.99 -5.00284 74.2661 -104.991 -90.0351
			      69.9982 -5.32433 19.9689 -0.035619 -3.906)
		   #f(12.7313 -0.091677 65.4018 70.0881 -110.296 -72.2013
			      -114.981 159.993 -5.00284 74.2612 -105.001 -90.0019
			      69.9982 -5.32433 20.0038 -0.029619 -3.906)
		   #f(14.1073 1.10352 52.3712 60.5681 -106.423 -87.0543 -108.372 159.898 -5.00284 74.2612 -105.001 -90.0019 69.9982 -5.72958 20.0038 -0.029619 -3.906)
		   )))
    (send *ri* :angle-vector-sequence avs (list 1000 1000 1000))
    (send *ri* :wait-interpolation)
;    (send *pr2* :head :look-at (send (send *pr2* :larm :end-coords) :worldpos))
;    (send *ri* :head-angle-vector (send *pr2* :angle-vector) 1000)
;    (send *ri* :wait-interpolation)
    (send goal :header :stamp (ros::time-now))
    (send goal :goal_id :stamp (ros::time-now))
    ;;  (send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC*)
    (send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*SLIP*)
    (send goal :goal :command :slip_trigger_magnitude 0.01)
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
 (let ((obj (car (send msg :objects))) (av (send *pr2* :angle-vector))
       (trans (send *tfl* :lookup-transform "/base_link"
	      "/narrow_stereo_optical_frame" (send msg :header :stamp))))
   (ros::ros-info "object detected.")
   (cond ((substringp "cotesys" (send obj :type))
	  (send *obj* :name "cotesys-cup"))
	 ((substringp "cmu" (send obj :type))
	  (send *obj* :name "cmu-cup"))
	 ((substringp "chen" (send obj :type))
	  (send *obj* :name "chen-cup")))
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (send *obj* :move-to *pr2* :world)
   (send *obj* :move-to trans :local)
;;   (send *obj* :move-to (send *pr2* :narrow_stereo_optical_frame :copy-worldcoords) :world)
   (send *obj* :move-to (ros::tf-pose->coords (send obj :pose)) :local)
   (send *pr2* :angle-vector av)
   ))

(defun init2 ()
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
  (ros::advertise "robotsound" sound_play::SoundRequest 5)
  (setq *obj* (body+ (make-cylinder 50 90)
					 (send (make-cube 70 10 80) :translate #f(45 0 45))))
  (send *obj* :translate-vertices (float-vector 50 0 0))
  (send *obj* :rotate-vertices pi/2 :y)
  (send *obj* :rotate-vertices (/ pi -4) :x)
  (when x::*display* (objects (list *pr2* *obj*)))
  )

(defun start-subscribe ()
  (ros::subscribe "/ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection))
(defun stop-subscribe ()
  (ros::subscribe "/ObjectDetection" posedetection_msgs::ObjectDetection #'identity))

(defun check (&key (timeout 10) (type nil))
  (start-subscribe) (unix::sleep 1)
  (send *obj* :move-to (make-coords) :world)
  (let ((pos1 (send (send *obj* :copy-worldcoords) :worldpos)) pos2 tm)
   (setq tm (ros::time-now))
   (while (ros::ok)
    (ros::spin-once)
    (setq pos2 (send *obj* :worldpos))
    (print (distance pos1 pos2))
    (when (and (< 0.01 (distance pos1 pos2))
	       (or (not type) (substringp type (send *obj* :name))))
      (ros::ros-info (format nil "ok - cup-type(~a)" (send *obj* :name)))
      (if x::*display* (send *irtviewer* :draw-objects))
      (stop-subscribe)
      (check-pcl)
      (return-from check t))
    (when (< timeout (send (ros::time- (ros::time-now) tm) :to-sec))
      (ros::ros-info "timeout")
      (stop-subscribe)
      (return-from check nil))
    (when x::*display*
      (send *viewer* :draw-objects)
      (x::window-main-one))
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
		 (if (and (not (equal 3dpos #f(0 0 0)))
			  (< (norm 3dpos) (norm minpos)))
		     (setq minpos 3dpos))
		 ))
	 (setq 3dpos minpos))
   (print (list 2dpos '-> 3dpos))
   (send *obj* :locate (send *pr2* :narrow_stereo_optical_frame :transform-vector 3dpos) :world)
   ;;
   (send *pr2* :angle-vector av)
   (when x::*display* (send *irtviewer* :draw-objects))
   ))

;;
;; for wiping table
;;
(defun wipe-table ()
  )