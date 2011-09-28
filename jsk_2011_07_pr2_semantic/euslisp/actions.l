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
                          :arg jp-str
			  :arg2 "aq_rm.phont")))

(defun pick (arm)
  (let* (tmp via1 target av vav avs)
    (ros::spin-once)
    (setq av (send *ri* :state :potentio-vector))
    (send *pr2* :angle-vector av)
    ;; calculate via point
    ;; tmp : object coords wrt robot coordinates
    ;; *pr2* :world, *obj* :world
    (setq tmp (make-coords :pos (send *pr2* :inverse-transform-vector (send *obj* :worldpos)))) ;; local
    (cond ((substringp "cup" (send *obj* :name))
	   (send tmp :translate #f(40 60 -50)) ;; move-to handle
	   (send tmp :newcoords
		 (rpy-matrix pi/2 pi/2 pi/2) ;; grasp from top
		 ;;(rpy-matrix -pi/2 0 0)        ;; grasp from side
		 (send (send tmp :copy-worldcoords) :worldpos)))
	  ((substringp "sandwich" (send *obj* :name))
	   (send tmp :newcoords
		 (rpy-matrix 0 0 0) ;; grasp straight
		 ;;(rpy-matrix 0 pi/2 pi/2) ;; grasp from top
		 ;;(rpy-matrix -pi/2 0 0)        ;; grasp from side
		 (send (send (send tmp :copy-worldcoords) :translate #f(50 100 100) :world) :worldpos)) ;; grasp baggage
		 ;;(send (send tmp :copy-worldcoords) :worldpos) ;; grasp center
		 ))
    (setq via1 (send tmp :copy-worldcoords))
    (send via1 :translate #f(-300 0 0) :world) ;; grasp from far
    ;;(send via1 :translate #f(0 0 150) :world) ;; grasp from top
    ;;(send via1 :translate #f(0 140 100) :world) ;; grasp from side
    (setq target (send tmp :copy-worldcoords))
    (ros::ros-warn (format nil "tmp ~a" tmp))
    (ros::ros-warn (format nil "via1 ~a" via1))
    (setq avs (send *ri* :move-end-coords-plan via1
		    :move-target arm :use-torso t))
    (when (null avs)
      (speak-jp "あいけいがとけませんでした")
      (cond ((send *pr2* :inverse-kinematics
		   via1 :move-tagret arm :use-torso t)
	     (send *ri* :angle-vector (send *pr2* :angle-vector) 5000))
	    (t
	     (return-from pick nil))))
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper arm 0.08 :wait t)
    ;; calculate grasp point
    (ros::spin-once)
    (setq vav (send *ri* :state :potentio-vector))
    (send *pr2* :angle-vector (copy-object vav))
    ;;
    (send target :transform *pr2* :parent)
    (print target)
    (send *pr2* :inverse-kinematics target :move-arm arm :rotation-axis t
	  :revert-if-fail nil)
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
  (pr2-tuckarm-pose :larm)
  (let ((client (instance ros::simple-action-client :init "/l_gripper_sensor_controller/event_detector" pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction))
	(goal (instance pr2_gripper_sensor_msgs::PR2GripperEventDetectorActionGoal :init))
	(av (send *ri* :state :potentio-vector))
	(avs (list #f(12 -0.253184 64.3257 69.9411 -121.542 49.5928 -114.592 160.001 -4.99809 74.2418 -105.001 -89.9936 70.002 -5.73317 20.0024 -3.73162 -18.6) #f(12 4.86278 57.2298 63.3719 -117.808 2.05774 -86.5717 162.479 -5.00284 74.2418 -104.991 -90.0019 69.9987 -5.73317 20.0024 -3.67162 -18.6) #f(12 10.0167 55.7951 52.3191 -99.0116 -76.7519 -49.7594 -96.8963 -4.99809 74.2418 -104.982 -90.0019 69.9953 -5.73067 20.0049 -11.9036 34.05) #f(12 8.77694 55.4801 51.2442 -79.8424 -118.059 -45.9951 9.59657 -4.99809 74.237 -105.001 -90.0019 69.992 -5.73067 20.0049 5.22038 36.426))))
    (send *ri* :angle-vector-sequence avs (list 1000 1000 1000 1000))
    (send *ri* :wait-interpolation)
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
       trans)
   (unless
       (send *tfl* :wait-for-transform 
	     "/base_link"
	     "/narrow_stereo_optical_frame" (send msg :header :stamp) 1)
     (ros::ros-error "could not solve /base_link to /narrow_stereo_optical_frame"))
   (setq trans (send *tfl* :lookup-transform "/base_link"
		     "/narrow_stereo_optical_frame" (send msg :header :stamp)))
   (ros::ros-info "object detected.")
   (send *obj* :name (send obj :type))
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
  (ros::advertise "/camera/rgb/points_sliced" sensor_msgs::PointCloud2 1)
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
  (ros::unsubscribe "/ObjectDetection"))

(defun check (&key (timeout 10) (type nil))
  (start-subscribe) (unix::sleep 1)
  (send *obj* :move-to (make-coords) :world)
  (let ((pos1 (send (send *obj* :copy-worldcoords) :worldpos))
	pos2 speak (tm (ros::time-now)))
   (while (ros::ok)
    (ros::spin-once)
    (setq pos2 (send *obj* :worldpos))
    (when (< 0.01 (distance pos1 pos2)) ;; found obj
      (cond ((or (not type) (substringp type (send *obj* :name))) ;; found target
	     (ros::ros-info (format nil "ok - obj-type(~a)" (send *obj* :name)))
	     (if x::*display* (send *irtviewer* :draw-objects))
	     (stop-subscribe)
	     (check-pcl)
	     (return-from check t))
	    (t
	     (ros::ros-info (format nil "no - obj-type(~a)" (send *obj* :name)))
	     (unless speak
	       (speak-jp "こっぷがみつかりました")
	       (setq speak t))
	     )))
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
 (let ((ray_srv "/pointcloud_screenpoint/screen_to_point")
       (av (send *pr2* :angle-vector))
       cam 2dpos req res 3dpos rot pos)
   (when (not (ros::wait-for-service ray_srv 5))
     (return-from check-pcl nil))
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
	     (setq res (ros::service-call ray_srv req))
	     (setq 3dpos (ros::tf-point->pos (send res :point)))
	     (if (and (not (equal 3dpos #f(0 0 0)))
		      (< (norm 3dpos) (norm minpos)))
		 (setq minpos 3dpos))
	     ))
	 (setq 3dpos minpos))
   ;;(ros::advertise "/camera/rgb/points_sliced" sensor_msgs::PointCloud2 1)
   (print (list 2dpos '-> 3dpos))
   (send *obj* :locate (send *pr2* :narrow_stereo_optical_frame :transform-vector 3dpos) :world)
   ;;
   (send *pr2* :angle-vector av)
   (when x::*display* (send *irtviewer* :draw-objects))
   ))

;;
;; for opening fridge
;;
(ros::load-ros-manifest "jsk_pr2_gui")
(load "package://jsk_pr2_gui/src/ipad_callback.l")
(ros::advertise "/ipad_marker_array" visualization_msgs::MarkerArray 10)
(defun open-fridge (hand handle &optional (rad pi/2))
  ;; 500mm is a distance between handle and axis
  (let ((a (send (send handle :copy-worldcoords) :translate #f(0 -500 200)))
	(b (send (send handle :copy-worldcoords) :translate #f(0 -500 -200)))
	(c handle) (di pi/2) (ra :z) ret avs tms uavs utms current-arm)
    (setq a (send a :worldpos) b (send b :worldpos) c (send c :worldpos))
    (setq ret (generate-circle-path c a b rad));;a,b can be reverse
    (show-markers (mapcar #'(lambda(x) (copy-object (send x :worldpos))) ret))
    (cond
     ((and (move-arm hand (send (send (send (elt ret 0) :copy-worldcoords) :translate #f(-100 0 0) :local) :rotate di :x) :rotation-axis :y :sec 1000)
	   (move-arm hand (send (send (send (elt ret 0) :copy-worldcoords) :translate #f(-100 0 0) :local) :rotate di :x) :rotation-axis t :sec 1000)))
     (t
      (ros::ros-warn "Could not solve ik, return from execute-open")
      (return-from open-fridge nil)))
    (send *ri* :move-gripper hand 0.09) ;; open
    (move-arm hand (send (send (send (elt ret 0) :copy-worldcoords) :translate #f(50 0 0) :local) :rotate di :x) :rotation-axis t :sec 1000)
    (send *ri* :move-gripper hand 0.0 :effort 100) ;; grasp
    (dolist (r (cdr ret))
      (setq rr (send r :copy-worldcoords))
      (when (send *pr2* :inverse-kinematics (send (send rr :copy-worldcoords) :rotate di :x) :move-arm hand :rotation-axis ra :debug-view nil)
	(push (send *pr2* :angle-vector) avs)
	(push 300 tms))
      )
    (send *ri* :angle-vector-sequence (reverse avs) tms)
    (send *ri* :wait-interpolation)
    ;; (send *ri* :move-gripper hand 0.09) ;; open
    ))

(defun open-fridge-by-marker (&key (pickup nil))
  (let ((handle (send *obj* :copy-worldcoords)) avs-tms ret)
    (setq *node-debug* nil)
    ;; open the door
    (send handle :rotate pi :z)
    (send handle :rotate pi/2 :y)
    (setq avs-tms (open-fridge :rarm handle))
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :angle-vector))
    (send *pr2* :larm :shoulder-r :joint-angle 80)
    (send *pr2* :head :look-at (v+ (send handle :worldpos) #f(100 -200 -100)))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;; pick object inside fridge
    (cond (pickup
	   (if (check :type pickup)
	       (setq ret (pick :larm))
	     (speak-jp "ほしいものはありません")))
	  (t
	   (setq ret t)))
    ;; close the door
    (send *ri* :move-gripper :rarm 0.09)
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :angle-vector))
    (send *pr2* :rarm :inverse-kinematics handle :rotation-axis nil)
    (send *ri* :rarm-angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper :rarm 0.0 :wait nil)
    (pr2-tuckarm-pose (if pickup :larm :rarm))
    ret ))

(defun demo-open-fridge (&key (go-back t) (pickup nil))
  (let (opose co ret)
    (pr2-tuckarm-pose :rarm)
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :torso :angle-vector #f(0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;
    (when go-back
      (setq opose (send (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)) :copy-worldcoords))
      (setq *opose* opose))
    (setq co (make-coords :pos #f(60936.374 6952.224 0)
			  :rpy #f(-0.580 0 0)))
    (send *ri* :move-to co)
    (let ((cur (send *tfl* :lookup-transform "/map" "/base_footprint" (ros::time 0)))
	  diff)
      (send *pr2* :move-to cur :world)
      (setq diff (v- (send co :worldpos) (send cur :worldpos)))
      (setq diff (send (send *pr2* :inverse-transformation) :rotate-vector diff))
      (send *ri* :go-velocity (/ (elt diff 0) 3000) (/ (elt diff 1) 3000)
	    (/ (elt (car (rpy-angle (send (send cur :transformation co) :worldrot))) 0) 3)
	    3000) ;; 3sec
      (unix::sleep 3))
    (send *pr2* :head :angle-vector #f(0 0))
    (send *ri* :head-angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (check :type "fridge")
    (send *pr2* :head :look-at (send *obj* :worldpos))
    (send *ri* :head-angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (setq ret (open-fridge-by-marker :pickup pickup))
    (send *pr2* :torso :angle-vector #f(0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (when go-back
      (send *ri* :move-to opose))
    ret ))
