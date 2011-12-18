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

;; calculate via point
;; tmp : object coords wrt robot coordinates
;; *pr2* :world, obj :world
(defun get-pick-handle (obj arm)
  (let (tmp via)
    (setq tmp (send *pr2* :transformation obj)) ;; local pose
    (cond ((substringp "cup" (send obj :name))
	   (send tmp :translate #f(40 60 -50) *pr2*) ;; move-to handle
	   (send tmp :newcoords
		 (rpy-matrix pi/2 pi/2 pi/2) ;; grasp from top
		 ;;(rpy-matrix -pi/2 0 0)        ;; grasp from side
		 (send (send tmp :copy-worldcoords) :worldpos))
	   (setq via (send tmp :copy-worldcoords))
	   (send via :translate #f(-300 0 0) *pr2*) ;; grasp from far
	   )
	  ((substringp "tea_box" (send obj :name)) ;; milktea_box
	   (send tmp :translate #f(0 0 20) :local)
	   (send tmp :rotate pi/2 :y)
	   (send tmp :rotate pi/2 :x)
	   (send tmp :rotate (/ pi 4) :y)
	   (push (send tmp :copy-worldcoords) via)
	   (send (car via) :translate #f(0 -50 100) obj) ;; grasp from far
	   (send (car via) :rotate (/ pi 4) :y)
	   (push (send tmp :copy-worldcoords) via)
	   (send (car via) :translate #f(200 -150 100) obj) ;; grasp from far
	   (send (car via) :rotate pi/2 :y)
	   (push (send (car via) :copy-worldcoords) via)
	   (send (car via) :translate #f(-200 0 -100) *pr2*) ;; grasp from far
	   (send (car via) :rotate pi/2 :y)
	   )
	  ((substringp "controller" (send obj :name))
	   (send tmp :translate #f(0 0 -40) *pr2*) ;; move-to handle
	   (send tmp :rotate pi/2 :y)
	   (send tmp :rotate pi/2 :x)
	   (setq via (send tmp :copy-worldcoords))
	   (send via :translate #f(0 0 200) *pr2*) ;; grasp from top
	   )
	  ((substringp "sandwich" (send obj :name))
	   (send tmp :newcoords
		 (rpy-matrix 0 0 0) ;; grasp straight
		 ;;(rpy-matrix 0 pi/2 pi/2) ;; grasp from top
		 ;;(rpy-matrix -pi/2 0 0)        ;; grasp from side
		 (send (send (send tmp :copy-worldcoords) :translate #f(50 100 100) *pr2*) :worldpos)) ;; grasp baggage
	   (setq via (send tmp :copy-worldcoords))
	   (send via :translate #f(-300 0 0) *pr2*) ;; grasp from far
	   ))
    (list via tmp)
    ))

(defun pick (obj arm)
  (let* (tmp via target av vav avs (success t))
    (setq tmp (get-pick-handle obj arm))
    (setq via (car tmp) tmp (cadr tmp))
    ;;
    (setq target (send tmp :copy-worldcoords))
    (ros::ros-warn (format nil "tmp ~a" tmp))
    (ros::ros-warn (format nil "via ~a" via))
    ;;
    (ros::spin-once)
    (setq av (send *ri* :state :potentio-vector))
    (send *pr2* :angle-vector av)
    ;; start move joints
    ;; call openrave planning and move
    (when (not (listp via))
      (setq avs (send *ri* :move-end-coords-plan via
		      :move-target arm :use-torso t))
      (unless avs
	(ros::ros-info "move arm planning was failed, in pick, actions.l")))
	;;(speak-jp "あい'/けい'が/とけませ'んでした。")))
    ;; if openrave is failed or not exists
    (when (null avs)
      (unless (listp via) (setq via (list via)))
      (send-all via :transform *pr2* :parent)
      (setq avs (list 
		 (mapcar #'(lambda(co)(send *pr2* arm :inverse-kinematics
			       co :use-torso t :revert-if-fail nil)
			     (send *pr2* :angle-vector)) via)
		 (make-list (length via) :initial-element 2000.0)))
      (cond ((every #'identity (car avs)) ;; ik all success ??
	     (send *ri* :angle-vector-sequence (car avs) (cadr avs)))
	    (t
	     (return-from pick nil))))
    ;;
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper arm 0.08 :wait t)
    ;;
    (ros::spin-once)
    (setq vav (send *ri* :state :potentio-vector))
    (send *pr2* :angle-vector (copy-object vav))
    ;;
    (send target :transform *pr2* :parent)
    (print target)
    (send *pr2* :inverse-kinematics target :move-arm arm :rotation-axis t
	  :revert-if-fail nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    ;;
    (when (< (send *ri* :start-grasp arm) 5) ;; 5[mm]
      (setq success nil))
    (unix::sleep 1)
    ;;
    (send *ri* :angle-vector vav 3000)
    (send *ri* :wait-interpolation)
    ;;
    (when (< (send *ri* :start-grasp arm) 5) ;; 5[mm]
      (setq success nil))
    (unix::sleep 1)
    ;; return to initial pose
    (if avs
	(send *ri* :angle-vector-sequence
	      (append (cdr (reverse (car avs))) (list av))
	      (reverse (cadr avs)))
      (send *ri* :angle-vector av 3000))
    (send *ri* :wait-interpolation)
    ;; finish
    success ))

(defun wait-for-hand-impact (arm &key (timeout 30))
  (let* ((action-name (format nil "/~c_gripper_sensor_controller/event_detector" (if (eq arm :larm) #\l #\r)))
         (client (instance ros::simple-action-client :init action-name pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction))
         (goal (instance pr2_gripper_sensor_msgs::PR2GripperEventDetectorActionGoal :init)))
    (unless (send client :wait-for-server 5)
      (return-from wait-for-hand-impact nil))
    (send goal :header :stamp (ros::time-now))
    (send goal :goal_id :stamp (ros::time-now))
    ;;(send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC*)
    (send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*FINGER_SIDE_IMPACT_OR_ACC*)
    ;;(send goal :goal :command :trigger_conditions pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand::*SLIP*)
    (send goal :goal :command :slip_trigger_magnitude 0.02)
    (send goal :goal :command :acceleration_trigger_magnitude 3.0) ;; m/s^2
    (send client :send-goal goal)
    (ros::ros-info "wait for touching robot hand")
    (send client :wait-for-result :timeout timeout))
  )

(defun hand-over (arm &key (wait-shock nil))
  (ros::spin-once)
  (let* ((av (send *ri* :state :potentio-vector))
	(tuckarm (check-tuckarm-pose))
	(isfreearm (eq arm tuckarm))
	;; this is for :larm
	(avs (list #f(12 0 64 70 -122 50 -115 160 -4 74 -105 -90 70 -5 20 2 15)
		   #f(12 6 9 106 -77 35 -124 -128 -4 75 -104 -89 70 0 20 3 30)
		   #f(12 13 21 62 -105 -117 -66 -71 -4 74 -104 -89 70 -5 20 4 40)
		   #f(12 9 24 50 -94 -158 -70 39 -4 74 -104 -89 70 -5 20 5 30)))
	(tms (make-list (length avs) :initial-element 1000))
	(l-r-reverse #f(1  -1 1 -1 1 -1 1 -1  -1 1 -1 1 -1 1 -1  -1 1)))
    ;;
    (if (eq arm :rarm)
	(setq avs
	      (mapcar #'(lambda(av)
			  (map float-vector #'*
			       (concatenate float-vector
					    (subseq av 0 1) (subseq av 8 15)
					    (subseq av 1 8) (subseq av 15 17))
			       l-r-reverse))
			  avs)))
    ;;
    (unless isfreearm
      (pr2-reset-pose)
      (setq avs (subseq avs (- (length avs) 2))
	    tms (subseq tms (- (length avs) 2))
	    av (send *ri* :state :potentio-vector))
      (setq tuckarm arm))
    ;;
    (send *ri* :angle-vector-sequence avs tms)
    (send *ri* :wait-interpolation)
    ;;
    (if (and wait-shock (not (numberp wait-shock))) ;; if wait-shock = t
	(setq wait-shock 10))
    (if wait-shock
	(progn (wait-for-hand-impact arm :timeout wait-shock)
	       (ros::ros-info "return from gripper sensor event")
	       (send *ri* :move-gripper arm 0.08 :wait t))
      (progn
	(send *ri* :move-gripper arm 0.08 :wait t)
	(unix::sleep 3)))
    ;;
    (send *ri* :angle-vector-sequence (append (cdr (reverse avs)) (list av)) tms)
    (send *ri* :move-gripper arm 0.00 :wait nil)
    (send *ri* :wait-interpolation)
    (pr2-tuckarm-pose tuckarm)
    ))

(defun objectdetection (msg)
 (let ((obj (car (send msg :objects))) (av (send *pr2* :angle-vector))
       trans)
   (unless
       (send *tfl* :wait-for-transform 
	     "/base_footprint"
	     "/narrow_stereo_optical_frame" (send msg :header :stamp) 1)
     (ros::ros-error "could not solve /base_footprint to /narrow_stereo_optical_frame"))
   (setq trans (send *tfl* :lookup-transform "/base_footprint"
		     "/narrow_stereo_optical_frame" (send msg :header :stamp)))
   (ros::ros-info "object detected.")
   (send *obj* :name (send obj :type))
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (send *obj* :move-to *pr2* :world)
   (send *obj* :move-to trans :local)
   (send *obj* :move-to (ros::tf-pose->coords (send obj :pose)) :local)
   (send *pr2* :angle-vector av)
   ))

(defun start-subscribe ()
  (ros::subscribe "/narrow_stereo/left/ObjectDetection"
		  posedetection_msgs::ObjectDetection #'objectdetection))
(defun stop-subscribe ()
  (ros::unsubscribe "/narrow_stereo/left/ObjectDetection"))

(defun init2 ()
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
  (ros::advertise "robotsound" sound_play::SoundRequest 5)
  (ros::advertise "/camera/rgb/points_sliced" sensor_msgs::PointCloud2 1)
  ;; *obj* is dummy object for detector callback
  ;;      :name <- frame_id, :pos,:rot <- pose
  (setq *obj* (body+ (make-cylinder 50 90)
		     (send (make-cube 70 10 80) :translate #f(45 0 45))))
  (send *obj* :translate-vertices (float-vector 50 0 0))
  (send *obj* :rotate-vertices pi/2 :y)
  (send *obj* :rotate-vertices (/ pi -4) :x)
  (when x::*display* (objects (list *pr2* *obj*)))
  )

(defun check (&key (timeout 10) (type nil))
  (start-subscribe) (unix::sleep 2)
  (send *obj* :move-to (make-coords) :world)
  (let ((pos1 (send (send *obj* :copy-worldcoords) :worldpos))
	pos2 speak (tm (ros::time-now)))
   (while (ros::ok)
    (ros::spin-once)
    (setq pos2 (send *obj* :worldpos))
    (when (< 0.01 (distance pos1 pos2)) ;; moved a little -> found obj
      (cond ((or (not (stringp type)) (substringp type (send *obj* :name))) ;; found target
	     (ros::ros-info (format nil "ok - obj-type(~a)" (send *obj* :name)))
	     (if x::*display* (send *irtviewer* :draw-objects))
	     (stop-subscribe)
	     ;; (check-pcl) ;; this is working only for cups 
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
 (let ((ray_srv "/pointcloud_screenpoint_nodelet/screen_to_point")
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
;;   *pr2* and *obj* are needed on worldcoords ??
(ros::load-ros-manifest "jsk_pr2_gui")
(load "package://jsk_pr2_gui/src/utils.l")
(ros::advertise "/ipad_marker_array" visualization_msgs::MarkerArray 10)
(defun open-fridge (hand handle &optional (rad pi/2))
  ;; 500mm is a distance between handle and axis
  (let ((a (send (send handle :copy-worldcoords) :translate #f(0 -500 200)))
	(b (send (send handle :copy-worldcoords) :translate #f(0 -500 -200)))
	(c handle) (di pi/2) (ra :z) ret avs tms uavs utms current-arm)
    (setq a (send a :worldpos) b (send b :worldpos) c (send c :worldpos))
    (setq ret (generate-circle-path c a b rad));;a,b can be reverse
    (show-markers (mapcar #'(lambda(x) (copy-object (send x :worldpos))) ret))
    (send *ri* :move-gripper hand 0.09 :wait nil) ;; open
    (setq *ret* (elt ret 0))
    (cond
     ((move-arm hand (send (send (send (elt ret 0) :copy-worldcoords) :translate #f(-100 0 0) :local) :rotate di :x) :rotation-axis t :sec 1000) t)
     (t
      (ros::ros-warn "Could not solve ik, return from execute-open")
      (return-from open-fridge nil)))
    (move-arm hand (send (send (send (elt ret 0) :copy-worldcoords) :translate #f(50 0 0) :local) :rotate di :x) :rotation-axis t :sec 1000)
    (when (< (send *ri* :start-grasp hand :gain 0.1) 10) ;; grasp
      (ros::ros-warn "Grasp handle failed, return from execute-open")
      (return-from open-fridge nil))
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
    (setq avs-tms (open-fridge :rarm handle (/ pi 3)))
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :angle-vector))
    (send *pr2* :larm :shoulder-r :joint-angle 80)
;;  (send *pr2* :head :look-at (v+ (send handle :worldpos) #f(100 -200 -100)))
    (send *pr2* :head :look-at (v+ (send handle :worldpos) #f(0 -400 -200)))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;; pick object inside fridge
    (cond (pickup
	   (dotimes (trial 3)
	     (if (check :type pickup)
		 (setq ret (pick *obj* :larm)))
	     (if ret (return)))
	   (unless ret (speak-jp "ほしいものはありません")))
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

(defun demo-open-fridge (co &key (go-back t) (pickup nil))
  (let (opose ret)
    (pr2-tuckarm-pose :rarm)
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :torso :angle-vector #f(0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    ;;
    (when go-back
      (setq opose (send *ri* :state :worldcoords))
      (setq *opose* (send opose :copy-worldcoords)))
    (send *ri* :move-to co)
    (send *pr2* :move-to (send *ri* :state :worldcoords) :world)
    ;;
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
      (send *ri* :move-to opose)
      (send *pr2* :move-to (send *ri* :state :worldcoords) :world))
    ret ))


(defun pickup-sandwich () ;; this method use larm to pickup -> rarm tuckarm
  (pr2-reset-pose)
  (let (av0 spose gpose avs avs2 tms tms2)
    (setq av0 #f(12 5 75 105 -90 -70 0 20 0 65 -80 -125 -50 -120 160 0 -22))
    (setq avs (list #f(100 -24 21 6 -118 -60 -91 175 -62 57 -68 -117 33 -79 170 0 4)
		    #f(100 -19 -20 11 -55 -170 -75 80 -62 57 -68 -117 33 -79 170 0 6)))
    (setq avs2 (list (float-vector 0 2 31 68 -145 33 -115 53 -63 57 -68 -117 33 -79 170 0 27)
		     (float-vector 0 44 28 129 -122 -4 -41 -70 -63 57 -68 -117 33 -79 170 0 27)
		     (float-vector 0 2 73 93 -92 9 -6 -34 -63 57 -68 -117 33 -79 170 0 27)))
    (setq tms2 (list 3000 2000 2000))
    ;;
    (print av0)
    (send *ri* :stop-grasp :larm)
    (send *pr2* :head :look-at (send (send (send *pr2* :copy-worldcoords) :translate #f(800 0 1500)) :worldpos))
    (send *ri* :head-angle-vector (send *pr2* :angle-vector) 1000)
    (speak-jp "さんどい'っちを/ください")
    (send *ri* :wait-interpolation)
    ;;
    (send *ri* :angle-vector-sequence avs (list 2000 2000))
    (send *ri* :wait-interpolation)

    (wait-for-hand-impact :larm :timeout 10)

    (send *ri* :move-gripper :larm 0.005 :effort 15 :wait t)
    (unix:sleep 1)
;    (send *ri* :angle-vector-sequence (append (cdr (reverse avs)) (list (car avs2))) tms)
    (send *ri* :angle-vector (car avs2) 2000)
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector-sequence (append (cdr avs2) (list av0)) tms2)
    (send *ri* :wait-interpolation)
    (speak-jp "あ'りがとう") (unix::sleep 5)
    ))

