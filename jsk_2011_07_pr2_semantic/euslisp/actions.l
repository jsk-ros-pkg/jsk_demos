#!/usr/bin/env roseus
(require :pr2-interface "package://pr2eus/pr2-interface.l")
;;(require :pr2eus-openrave "package://pr2eus_openrave/pr2eus-openrave.l")

(ros::roseus-add-msgs "posedetection_msgs")
(ros::roseus-add-msgs "pr2_gripper_sensor_msgs")
(ros::roseus-add-srvs "jsk_recognition_msgs")

;(ros::roseus "grasp_cup")

;; calculate via point
;; tmp : object coords wrt robot coordinates
;; *pr2* :world, obj :world
(defun get-pick-handle (obj arm)
  (let (tmp via)
    (setq tmp (send *pr2* :transformation obj)) ;; robot relative pose
    (cond ((substringp "cup" (send obj :name))
	   (send tmp :translate #f(40 30 -50) :world) ;; move-to handle
	   (send tmp :newcoords
		 (rpy-matrix pi/2 pi/2 pi/2) ;; grasp from top
		 ;;(rpy-matrix -pi/2 0 0)        ;; grasp from side
		 (send (send tmp :copy-worldcoords) :worldpos))
	   (setq via (send tmp :copy-worldcoords))
	   (send via :translate #f(0 30 150) :world) ;; grasp from top
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
	   (send tmp :translate #f(0 0 -60) :world) ;; move-to handle
	   (send tmp :rotate pi/2 :y)
	   (send tmp :rotate pi/2 :x)
	   (setq via (send tmp :copy-worldcoords))
	   (send via :translate #f(0 0 200) :world) ;; grasp from top
	   )
	  ((substringp "sandwich" (send obj :name))
	   (send tmp :newcoords
		 (rpy-matrix 0 0 0) ;; grasp straight
		 ;;(rpy-matrix 0 pi/2 pi/2) ;; grasp from top
		 ;;(rpy-matrix -pi/2 0 0)        ;; grasp from side
		 (send (send (send tmp :copy-worldcoords) :translate #f(50 100 100) *pr2*) :worldpos)) ;; grasp baggage
	   (setq via (send tmp :copy-worldcoords))
	   (send via :translate #f(-300 0 0) :world) ;; grasp from far
	   ))
    (list via tmp)
    ))

(defun pick (obj arm &key (force nil))
  (let* (tmp via target av vav avs (success t))
    (setq tmp (get-pick-handle obj arm))
    (setq via (car tmp) tmp (cadr tmp))
    ;;
    (setq target (send tmp :copy-worldcoords))
    (ros::ros-warn (format nil "tmp ~a" tmp))
    (ros::ros-warn (format nil "via ~a" via))
    ;;
    (ros::spin-once)
    (send *pr2* :move-to (send *ri* :state :worldcoords) :world)
    (setq av (send *ri* :state :potentio-vector))
    (send *pr2* :angle-vector av)

    ;; start move joints
    ;; call openrave planning and move
    (when (not (listp via))
      (setq avs (send *ri* :move-end-coords-plan via
		      :move-arm arm :use-torso t))
      (unless avs
	(ros::ros-info "move arm planning was failed, in pick, actions.l")
	(speak-jp "しょうとつかいひどうさしっぱい")))
    (when (not (or force avs)) (return-from pick nil))

    ;; if openrave is failed or not exists
    (when (null avs)
      (unless (listp via) (setq via (list via)))
      (send-all via :transform *pr2* :parent)
      (setq avs (list 
		 (mapcar #'(lambda(co)(send *pr2* arm :inverse-kinematics
			       co :revert-if-fail nil)
			     (send *pr2* :angle-vector)) via)
		 (make-list (length via) :initial-element 2000.0)))
      (cond ((every #'identity (car avs)) ;; ik all success ??
	     (send *ri* :angle-vector-sequence (car avs) (cadr avs)))
	    (t
	     (return-from pick nil))))
    ;;
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper arm 0.09 :wait t)
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
    ;;
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (pr2-tuckarm-pose tuckarm)
    ))

(defmethod coordinates
  (:stamp (&optional s) (if s (setf (get self :stamp) s)) (get self :stamp)))
(defun objectdetection (force msg)
 (let ((obj (car (send msg :objects)))
       (frame (send msg :header :frame_id)) stamp tstamp trans)
   (when (or (null *target-name*)
	     (substringp *target-name* (send obj :type)))
     (setq stamp (if force (ros::time 0) (send msg :header :stamp)))
     (unless
	 (send *tfl* :wait-for-transform
	       "/base_footprint" frame stamp 1)
       (ros::ros-error "could not solve /base_footprint to ~A" frame))
     (setq trans (send *tfl* :lookup-transform "/base_footprint" frame stamp))
     (when trans
       (ros::ros-info "object detected ~A ~A ~20,9f" (send obj :type)
		      (ros::tf-pose->coords (send obj :pose))
		      (send (send msg :header :stamp) :to-sec))
       (send *obj* :name (send obj :type))
       (send *obj* :stamp (send msg :header :stamp))
       (send *obj* :move-to *pr2* :world)
       (send *obj* :move-to trans :local)
       (send *obj* :move-to (ros::tf-pose->coords (send obj :pose)) :local)
       ))
   ))

(defun start-subscribe (tf-force)
  (ros::subscribe "/narrow_stereo/left/ObjectDetection"
		  posedetection_msgs::ObjectDetection #'objectdetection tf-force))
(defun stop-subscribe ()
  (ros::unsubscribe "/narrow_stereo/left/ObjectDetection"))

(defun init2 ()
  (unless (boundp '*pr2*) (pr2))
  (unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
  (ros::advertise "/camera/rgb/points_sliced" sensor_msgs::PointCloud2 1)
  ;; *obj* is dummy object for detector callback
  ;;      :name <- frame_id, :pos,:rot <- pose
  (setq *target-name* nil)
  (setq *obj* (body+ (make-cylinder 50 90)
		     (send (make-cube 70 10 80) :translate #f(45 0 45))))
  (send *obj* :translate-vertices (float-vector 50 0 0))
  (send *obj* :rotate-vertices pi/2 :y)
  (send *obj* :rotate-vertices (/ pi -4) :x)
  (when (and x::*display* (/= x::*display* 0)) (objects (list *pr2* *obj*)))
  )

(ros::advertise "/object_detection_marker_array" visualization_msgs::MarkerArray 5)

(defun check (&key (timeout 10) (type nil) (tf-force nil))
  (setq *target-name* type) 
  (start-subscribe tf-force) (unix::sleep 2)
  (send *obj* :move-to (make-coords) :world) ;; reset obj position
  (let* (current-object-coords previous-object-coords
         speak (start-time (ros::time-now)) (detect-object-list))
   (ros::ros-info "check (timeout=~A, type=~A, tf-force=~A)" timeout type tf-force)
   (while (ros::ok)
    (ros::spin-once) ;; *obj* was updated in objectdetection

    ;; check *obj* is previously detected
    (when (and (> (send (send *obj* :stamp) :to-sec)
		  (send start-time :to-sec))                          ;; discard object detection result before check function called
               (not (member (send *obj* :stamp) detect-object-list))) ;; check if obj is already stored in detect-object-list
      (push (send *obj* :stamp) detect-object-list)
      (setq current-object-coords (send *obj* :copy-worldcoords)) ;; updated position

      (ros::publish "/object_detection_marker_array"
                    (instance visualization_msgs::MarkerArray :init
                              :markers
                              (list (text->marker-msg (send *obj* :name)
                                                      (send (send current-object-coords :copy-worldcoords)
                                                            :translate #f(-100 0 100))
                                                      (instance std_msgs::header :init
								:stamp (send *obj* :stamp)
								:frame_id "/base_footprint")
                                                      :ns "object_text" :color #f(1 0 0)
                                                      :scale 100 :lifetime 15 :id 0)
                                    (coords->marker-msg current-object-coords
                                                        (instance std_msgs::header :init
								  :stamp (send *obj* :stamp)
								  :frame_id "/base_footprint")
                                                        :ns  "object_coords"
                                                        ;; :size 200 :width 20
                                                        :lifetime 15 :id 1)
                                    )))

      (ros::ros-info "object detected ~A diff-pos:~A, diff-rot:~A"
                     (send *obj* :name)
                     (if previous-object-coords
			 (norm (send previous-object-coords :difference-position current-object-coords)))
                     (if previous-object-coords
			 (elt (send previous-object-coords :difference-rotation current-object-coords
				    :rotation-axis :xy) 2)))
      (when (and previous-object-coords
		 (< (norm (send previous-object-coords :difference-position current-object-coords)) 10.0)) ;; moved a little -> found obj
        (cond
	 ((or (not (stringp type)) (substringp type (send *obj* :name))) ;; found target
	  (ros::ros-info (format nil "ok - obj-type(~a)"  *obj*))
	  (if (and x::*display* (/= x::*display* 0)) (send *irtviewer* :draw-objects))
	  (stop-subscribe)
	  (unless speak
	    (speak-jp (format nil "~a がみつかりました" (send *obj* :name)))
	    (setq speak t))
	  (cond ((substringp "cup" (send *obj* :name))
		 ;; this is working only for cups 
		 (when (check-pcl)
		   (return-from check t)))
		(t
		 (return-from check t))))
	 (t
	  (ros::ros-info (format nil "no - obj-type(~a)" (send *obj* :name)))
	  ))
	) ;; when (and pos1...
      (when (and x::*display* (/= x::*display* 0))
        (send *irtviewer* :draw-objects)
        (x::window-main-one))
      (setq previous-object-coords current-object-coords) ;; set current coords ad previous coords
      ) ;; memq *obj* detect-object-list
    (when (< timeout
	     (send (ros::time- (ros::time-now) start-time) :to-sec))
      (ros::ros-info "timeout")
      (stop-subscribe)
      (return-from check nil))
    (unix::usleep (* 50 1000))
    ) ;; while (ros::ok)
   ))

(defun check-pcl ()
 (let ((ray_srv "/pointcloud_screenpoint_nodelet/screen_to_point")
       (av (send *pr2* :angle-vector))
       cam 2dpos req res 3dpos rot pos ret)
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
	     (setq req (instance jsk_recognition_msgs::TransformScreenpointRequest :init
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
   (when (< (distance 3dpos (send *obj* :worldpos)) 100)
     (send *obj* :locate (send *pr2* :narrow_stereo_optical_frame :transform-vector 3dpos) :world)
     (setq ret t))
   ;;
   (send *pr2* :angle-vector av)
   (when (and x::*display* (/= x::*display* 0)) (send *irtviewer* :draw-objects))
   t ))

;;
;; for opening fridge
;;   *pr2* and *obj* are needed on worldcoords ??
;;(ros::load-ros-manifest "jsk_ipad_gui")
;;(load "package://jsk_ipad_gui/src/utils.l")
(ros::load-ros-manifest "jsk_smart_gui")
(load "package://jsk_smart_gui/src/utils.l")
(ros::advertise "/tablet_marker_array" visualization_msgs::MarkerArray 10)
(defun open-fridge (hand handle &optional (rad pi/2)
                         &key (radius 500) ((:rotation-axis ra) :z) (use-torso 0.0025)
                         (return-sequence) (time-tick 300) (wait-time 4)
                         (path-div 20) (grasp-check t))
  ;; 500mm is a distance between handle and axis
  (let ((acds (send (send handle :copy-worldcoords) :translate (float-vector 0 (- radius) 200)))
        (bcds (send (send handle :copy-worldcoords) :translate (float-vector 0 (- radius) -200)))
        (di -pi/2) ret avs tms uavs utms current-arm)
    (setq ret (generate-circle-path (send handle :worldpos)
                                    (send acds :worldpos) (send bcds :worldpos)
                                    rad :div path-div));;a,b can be reverse
    (show-markers (mapcar #'(lambda(x) (copy-object (send x :worldpos))) ret))
    (send *ri* :move-gripper hand 0.09 :wait nil) ;; open
    ;;
    (mapc #'(lambda (x) (send x :rotate -pi/2 :x)) ret)
    (setq *ret* (elt ret 0))
    (cond
     ((move-arm hand (send (send (car ret) :copy-worldcoords)
                           :translate #f(-100 0 0) :local)
                :rotation-axis t :sec 1000) t)
     (t
      (ros::ros-warn "Could not solve ik, return from execute-open")
      (return-from open-fridge nil)))
    (move-arm hand (send (send (car ret) :copy-worldcoords)
                               :translate #f(50 0 0) :local)
              :rotation-axis t :sec 1000)
    (let ((grasp-ret (send *ri* :start-grasp hand :gain 0.1)))
      (when (and grasp-check
                 (< grasp-ret 8)) ;; grasp
        (ros::ros-warn "Grasp handle failed, return from execute-open")
        (return-from open-fridge nil)))
    (dolist (r (cdr ret))
      (setq rr (send r :copy-worldcoords))
      (let ((ik-solved
             (send *pr2* :inverse-kinematics
                   (send rr :worldcoords)
                   :use-torso use-torso
                   :move-arm hand :rotation-axis ra :debug-view nil :warnp nil)))
        (unless (or ik-solved (eq ra nil))
          (warn ";; retry ~A~%" rr);
          (setq ik-solved
                (send *pr2* :inverse-kinematics
                      (send rr :copy-worldcoords)
                      :use-torso use-torso
                      :move-arm hand :rotation-axis nil :debug-view nil :warnp nil)))
        (unless ik-solved (warn ";; failed ~A~%" rr))
        (when ik-solved
          (push (send *pr2* :angle-vector) avs)
          (push time-tick tms))
        ))
    (when return-sequence
      (return-from open-fridge (list (reverse avs) tms)))
    (send *ri* :angle-vector-sequence (reverse avs) tms)
    (cond
     ((integerp wait-time)
      (unix:sleep wait-time))
     ((numberp wait-time)
      (unix::usleep
       (round (* wait-time 1000 1000))))
     (wait-time
      (send *ri* :wait-interpolation)))
    (send *ri* :stop-grasp hand)
    (send *ri* :wait-interpolation)
    ;;
    (unix::sleep 1) (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq end-pt (send *pr2* hand :end-coords :worldpos))
    (setq idx (position (find-extream ret #'(lambda(x)(distance (send x :worldpos) end-pt)) #'<) ret))
    (ros::ros-info "idx = [~a/~a]" idx (length ret))
    (when (< idx (/ (length ret) 2))
      (return-from open-fridge nil))
    t))

(defun open-fridge-by-marker (&key (pickup nil))
  (let ((handle (send *obj* :copy-worldcoords)) (ret :not-open))
    (setq *node-debug* nil)
    ;; open the door
    (when (open-fridge :rarm handle (/ pi 3))
      ;;
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
      )
    ;; close the door
    (send *ri* :move-gripper :rarm 0.09)
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :angle-vector))
    (send *pr2* :rarm :inverse-kinematics handle :rotation-axis nil)
    (send *ri* :rarm-angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *ri* :move-gripper :rarm 0.0 :wait nil)
    (if (eq ret :not-open)
	(pr2-tuckarm-pose :rarm)
      (pr2-tuckarm-pose (if pickup :larm :rarm)))
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
    (dotimes (trial 3)
      (send *pr2* :head :angle-vector #f(0 0))
      (send *ri* :head-angle-vector (send *pr2* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      (check :type "fridge")
      (send *pr2* :head :look-at (send *obj* :worldpos))
      (send *ri* :head-angle-vector (send *pr2* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      (setq ret (open-fridge-by-marker :pickup pickup))
      (unless (eq ret :not-open)
	(setq ret nil)
	(return)))
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
    (speak-jp "さんどいっちをください")
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
    (speak-jp "ありがとおございます") (unix::sleep 5)
    ))

