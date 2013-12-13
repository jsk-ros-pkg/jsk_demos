(setq *obj* (make-sphere 100)) ;; dummy object
(setq *target-name* nil)
(defparameter *detection-topic* "/openni/rgb/ObjectDetection")
(load "package://jsk_perception/euslisp/detection_interface.l")

(ros::load-ros-manifest "jsk_smart_gui")
(require :jsk-smart-gui-utils "package://jsk_smart_gui/src/utils.l")

(defvar *use-voicetext* t)
(defvar *use-english* nil)

(if (and (boundp '*use-voicetext*) *use-voicetext*)
    (cond
     ((and (boundp '*use-english*) *use-english*)
      (setq *frige-speak-str* "the refrigerator"))
     (t (setq *frige-speak-str* "冷蔵庫")))
  (setq *frige-speak-str* "れいぞうこ"))

;;
;; action-utility
;;

(defun pick (pos &key (arm :larm)
			 (debug-view nil))
  (let ((tmp-pos (send pos :copy-worldcoords)))
    (format t  "Start Pick: ~a~%" pos)
    (send *ri* :stop-grasp arm)  ;; Stop grasp
    (send *pr2* :inverse-kinematics 
		  (progn
			(send tmp-pos :rotate  (deg2rad 90) :y) ;; alignment
			(send tmp-pos :translate #f(-25 0 0)))  ;; avoid collision
		  :rotation-axis :x
		  :move-target (send *pr2* arm :end-coords)
		  :link-list (send *pr2* :link-list
						   (send *pr2* arm :end-coords :parent)
						   (send *pr2* :torso_lift_link))
		  :debug-view nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)

    ;; Low down to pick 
    (send *pr2* :torso :waist-z
		  :joint-angle (- (send *pr2* :torso :waist-z :joint-angle) 30))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)
    (send *ri* :start-grasp arm)
    ;;(send *ri* :move-gripper arm 0.0 :effort 100 :wait t)
    (print "End Pick!"))
  )

(defun pick2 (pos &key (arm :rarm)
			  (rotation-axis :z)
			  (debug-view nil))
  (let ((tmp-pos (send pos :copy-worldcoords)))
    (format t  "Start Pick: ~a~%" pos)
    (send *ri* :stop-grasp arm)  ;; Stop grasp
    (send *pr2* arm :inverse-kinematics 
		  tmp-pos
		  :rotation-axis rotation-axis
		  :debug-view nil)
	(send *pr2* arm :move-end-pos (float-vector -100 0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)

	(send *pr2* arm :move-end-pos (float-vector 150 0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)

;    (send *pr2* :torso :waist-z
;		  :joint-angle (+ (send *pr2* :torso :waist-z :joint-angle) 230))
;    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
;    (send *ri* :wait-interpolation)
;    (unix::usleep 100000)
    (send *ri* :start-grasp arm)
    ;;(send *ri* :move-gripper arm 0.0 :effort 100 :wait t)
    (print "End Pick!"))
  )

(defun place2 (pos &key (arm :rarm)
			  (rotation-axis :z)
			  (debug-view nil))
  (let ((tmp-pos (send pos :copy-worldcoords)))
	(send *pr2* arm :inverse-kinematics
		  tmp-pos
		  :rotation-axis rotation-axis
		  :debug-view debug-view)
	(send *pr2* :torso :waist-z :joint-angle
		  (+ (send *pr2* :torso :waist-z :joint-angle) 100))
	(send *ri* :angle-vector (send *pr2* :angle-vector))
	(send *ri* :wait-interpolation)
	
	(send *pr2* :torso :waist-z :joint-angle
		  (- (send *pr2* :torso :waist-z :joint-angle) 100))
	(send *ri* :angle-vector (send *pr2* :angle-vector))
	(send *ri* :wait-interpolation)
	(send *ri* :stop-grasp arm)
	(print "End Place!"))
  )

(defun place (pos &key (arm :larm)
			  (height -50)
			  (debug-view nil))
  (let ((tmp-pos (send pos :copy-worldcoords)))
    (format t  "Start Place: ~a~%" pos)
    (send *pr2* :inverse-kinematics 
	  (progn 
	    (send tmp-pos :rotate (deg2rad 90) :y)
	    (send tmp-pos :translate (float-vector height 0.0 0.0)))
	  :rotation-axis :x  ;; 
	  :move-target (send *pr2* arm :end-coords)
	  :link-list (send *pr2* :link-list
			   (send *pr2* arm :end-coords :parent)
			   (send *pr2* :torso_lift_link))
	  :debug-view debug)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)
    
    (send *pr2* :torso :waist-z
     	  :joint-angle (- (send *pr2* :torso :waist-z :joint-angle) 30))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)

    (send *ri* :stop-grasp arm)
    
    (send *pr2* :torso :waist-z
	  :joint-angle (- (send *pr2* :torso :waist-z :joint-angle) -50))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (unix::usleep 100000)
    (print "End Place!"))
  )

(defmacro start-detect-tabletop-object ()
  `(progn
	 (if (not (boundp '*tfb*)) (setq *tfb* (instance ros::transform-broadcaster :init)))
	 (defun tabletop-objectdetection-cb (msg)
	   (let ((frame-id (send msg :header :frame_id)) atype ret)
		 (print msg)
		 (unless (eq (char frame-id 0) #\/) (setq frame-id (concatenate string "/" frame-id)))
		 (dolist (obj-pose (send msg :objects))
		   (setq atype (send obj-pose :type))
		   (unless (eq (char atype 0) #\/) (setq atype (concatenate string "/" atype)))
		   (setq ret (ros::tf-pose->coords (send obj-pose :pose)))
		   (send *tfb* :send-transform ret frame-id atype)
		   (ros::ros-info "~A ~A ~A" ret frame-id atype))
		 ))
	 (ros::subscribe "ObjectDetection" posedetection_msgs::ObjectDetection #'tabletop-objectdetection-cb))
  )


(defun stop-detect-tabletop-object ()
  (ros::unsubscribe "ObjectDetection"))

(defun detect-tabletop-object (&optional (root-frame-id "/base_footprint"))
  (ros::spin-once)
  (if (not (boundp '*tfl*)) (setq *tfl* (instance ros::transform-listener :init)))
  (let (c obj-frame-id ret)
    (dolist (obj-frame-id (list "/tabletop0"
								"/tabletop1"
								"/tabletop2"
								"/tabletop3"
								"/tabletop4"))
      ;; advertise
      (when (send *tfl* :get-latest-common-time root-frame-id obj-frame-id)
		(setq c (send *tfl* :lookup-transform root-frame-id obj-frame-id (ros::time 0)))
		(when c
		  (ros::ros-info "~A ~A" obj-frame-id c)
		  (push c ret))))
	(ros::sleep)
    ret))

(defparameter *detection-loop-threshold* 0)
(warn ";; define detect-with-base-laser")
(defun detect-with-base-laser (obj obj-type &key (arg nil) (debug nil) (speak nil) (publish-marker nil))
  (ros::ros-info "detect-with-base-laser~%")
  (unless (send *ri* :joint-action-enable)
	(if obj (ros::ros-warn "~A detection failed" obj))
    (return-from detect-with-base-laser nil)
    )
  (let ((loop-flag t) (loop-counter 0) (obj-coords nil) (ret))
    (ros::roseus "objectdetection_publisher")
    (defparameter *detection-topic* "/narrow_stereo/left/ObjectDetection")
    (ros::rate 10)
    (ros::ros-info "detecting_now")
    (while loop-flag
      (setq ret (check-detection :type obj-type ;; work for any object
                                 :speak-name (if speak speak "")
                                 :target-object obj
                                 :publish-objectdetection-marker publish-marker
                                 :speak speak))
      (ros::spin-once)
      (if ret
          (progn
            (ros::ros-info "object pos info ~A before trans" obj)
            (send obj :translate (send *pr2* :worldpos) :world)
            (ros::ros-info "object pos info ~A after trans" obj)
            (send *pr2* :head :look-at
				  (if spot
					  (send (send obj spot) :worldpos)
					  (send obj :worldpos)))
            (send *ri* :angle-vector (send *pr2* :angle-vector))
            (push (send obj :worldcoords) obj-coords)
            ))

      ;;wait a few loop
      (when (> loop-counter *detection-loop-threshold*)
        (setq loop-flag nil)	;;temp method
        )
      (inc loop-counter)
      (ros::ros-info "~d loop counter ----" loop-counter)
      (ros::sleep)
      )
    )
  )

(warn ";; define hold-chair")
(defun hold-chair (chair)
  (hold-chair-prepair chair)
  (send *pr2* :inverse-kinematics (send (send *pr2* :larm :end-coords :copy-worldcoords) :translate #f(-300 0 0) *pr2*) :move-arm :larm :use-torso nil :rotation-axis :y)
  (send *pr2* :inverse-kinematics (send (send *pr2* :rarm :end-coords :copy-worldcoords) :translate #f(-300 0 0) *pr2*) :move-arm :rarm :use-torso nil :rotation-axis :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :rarm :angle-vector #f(-14.3227 16.0059 -85.3758 -102.121 -35.757 -32.3901 -173.201))
  (send *pr2* :larm :angle-vector #f(8.64394 19.7097 109.218 -85.0018 43.0966 -7.98218 163.739))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun hold-chair-prepair (chair)
  (send *pr2* :larm :angle-vector #f(25 0 0 -130 0 0 0))
  (send *pr2* :rarm :angle-vector #f(-25 0 0 -130 0 0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send *pr2* :reset-pose)
  (send *pr2* :head :look-at (vector-mean (send-all (send chair :handle) :worldpos)) :world)
  (send-message *pr2* euscollada-robot
				:inverse-kinematics (send (send (send (send chair :handle-larm) :copy-worldcoords) :rotate -pi/2 :x) :translate #f(30 100 200))
				:rthre (deg2rad 5)
				:thre 10
				:stop 300
				:weight #f(0 1 1 1 1 1 1 0)
				:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
				:move-target (send *pr2* :larm :end-coords)
				:rotation-axis :y)
  (send-message *pr2* euscollada-robot
				:inverse-kinematics (send (send (send (send chair :handle-rarm) :copy-worldcoords) :rotate pi/2 :x) :translate #f(30 -100 200))
				:rthre (deg2rad 5)
				:thre 10
				:stop 300
				:weight #f(0 1 1 1 1 1 1 0)
				:link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
				:move-target (send *pr2* :rarm :end-coords)
				:rotation-axis :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (send-message *pr2* euscollada-robot
				:inverse-kinematics (send (send (send (send chair :handle-larm) :copy-worldcoords) :rotate -pi/2 :x) :translate #f(30 100 -120))
				:rthre (deg2rad 5)
				:thre 10
				:stop 300
				:weight #f(0 1 1 1 1 1 1 0)
				:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
				:move-target (send *pr2* :larm :end-coords)
				:rotation-axis :y)
  (send-message *pr2* euscollada-robot
				:inverse-kinematics (send (send (send (send chair :handle-rarm) :copy-worldcoords) :rotate pi/2 :x) :translate #f(30 -100 -120))
				:rthre (deg2rad 5)
				:thre 10
				:stop 300
				:weight #f(0 1 1 1 1 1 1 0)
				:link-list (send *pr2* :link-list (send (send *pr2* :rarm :end-coords) :parent))
				:move-target (send *pr2* :rarm :end-coords)
				:rotation-axis :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

(defun open-fridge-traj (hand handle &optional (rad/length pi/2) &rest args
                         &key (radius 500) (path-div 20) (door-type :circle) &allow-other-keys)
  (let ((cds-traj
         (case door-type
           (:circle
            (open-fridge-circle-traj handle rad/length :radius radius :path-div path-div))
           (:slide
            (open-fridge-slide-traj handle rad/length :path-div path-div))
           )))
    (apply #'move-fridge-traj hand cds-traj args)
    ))
(defun open-fridge-slide-traj
  (handle &optional (path-length 200)
          &key  (path-div 20) (pre-start-x -100) (start-x 25)
          &allow-other-keys)
  (let ((tcds (send (send handle :copy-worldcoords) :translate (float-vector (- path-length) 0 0)))
        ret)
    (setq ret (generate-slide-path (send handle :worldpos)
                                   (send handle :worldpos) (send tcds :worldpos)
                                   path-length :div path-div))
    (mapc #'(lambda (x) (send x :rotate pi/2 :x)) ret)
    (setq *ret*
          (append (list (send (send (car ret) :copy-worldcoords)
                              :translate (float-vector pre-start-x 0 0) :local)
                        (send (send (car ret) :copy-worldcoords)
                              :translate (float-vector start-x 0 0) :local))
                  ret))
    ;; stop grasp
    ;; move (elt ret 0)
    ;; move (elt ret 1)
    ;; start grap
    ;; grasp check
    ;; move-trajectory (subseq ret 2)
    ;; finalize
    *ret*))

(defun open-fridge-circle-traj
  (handle &optional (rad pi/2)
          &key (radius 500) (path-div 20) (pre-start-x -100) (start-x 25))
  ;; 500mm is a distance between handle and axis
  (let ((acds (send (send handle :copy-worldcoords) :translate (float-vector 0 (- radius) 200)))
        (bcds (send (send handle :copy-worldcoords) :translate (float-vector 0 (- radius) -200)))
        (di -pi/2) ret)
    (setq ret (generate-circle-path (send handle :worldpos)
                                    (send acds :worldpos) (send bcds :worldpos)
                                    rad :div path-div));;a,b can be reverse
    (mapc #'(lambda (x) (send x :rotate -pi/2 :x)) ret)
    (setq *ret*
          (append (list (send (send (car ret) :copy-worldcoords)
                              :translate (float-vector pre-start-x 0 0) :local)
                        (send (send (car ret) :copy-worldcoords)
                              :translate (float-vector start-x 0 0) :local))
                  ret))
    ;; stop grasp
    ;; move (elt ret 0)
    ;; move (elt ret 1)
    ;; start grap
    ;; grasp check
    ;; move-trajectory (subseq ret 2)
    ;; finalize
    *ret*))

(defun move-fridge-traj (hand cds-traj
                              &key ((:rotation-axis ra) :z) (use-torso 0.0025)
                              (fix-waist-z 150) (return-sequence) (time-tick 300) (wait-time 4)
                              (grasp-check t)  (move-robot t) (start-sec 1000) &allow-other-keys)
  (let ((st0 (car cds-traj))
        (st1 (cadr cds-traj))
        (lst (cddr cds-traj))
        avs tms)
    (when move-robot  ;; open-gripper
      (send *ri* :move-gripper hand 0.09 :wait nil))
    (when fix-waist-z
      (unless (numberp fix-waist-z) (setq fix-waist-z 130))
      (send *pr2* :torso :waist-z :joint-angle fix-waist-z))
    (cond
     ((move-arm hand st0
                :use-torso (if use-torso (/ use-torso 5))
                :rotation-axis t :sec start-sec :move-robot move-robot)
      t)
     (t
      (ros::ros-warn "Could not solve ik, return from execute-open")
      (return-from move-fridge-traj nil)))

    (move-arm hand st1
              :use-torso (if use-torso (/ use-torso 5))
              :rotation-axis t :sec start-sec :move-robot move-robot)

    (let ((grasp-ret (if move-robot (send *ri* :start-grasp hand :gain 0.1) 10)))
      (when (and grasp-check
                 (< grasp-ret 8)) ;; grasp
        (ros::ros-warn "Grasp handle failed, return from execute-open")
        (return-from move-fridge-traj nil)))

    (dolist (rr lst)
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
      (return-from move-fridge-traj (list (reverse avs) tms)))

    (when move-robot
      (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
      (send *ri* :angle-vector-sequence (reverse avs) tms))
    ;; finalize
    (cond
     ((integerp wait-time)
      (send *ri* :ros-wait wait-time :spin-self t)) ;; attention-check ...
     ((numberp wait-time)
      (send *ri* :ros-wait wait-time :spin-self t) ;; attention-check ...
      ;;(unix::usleep
      ;;(round (* wait-time 1000 1000)))
      )
     (wait-time
      (send *ri* :wait-interpolation)))
    (send *ri* :stop-grasp hand)
    (send *ri* :wait-interpolation)
    ;;
    (send *ri* :ros-wait 1.0 :spin t :spin-self t) ;; attention-check ...
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (let ((end-pt (send *pr2* hand :end-coords :worldpos))
          idx)
      (setq idx (position (find-extream lst #'(lambda(x) (distance (send x :worldpos) end-pt)) #'<) lst))
      (ros::ros-info "idx = [~a/~a]" idx (length lst))
      (when (< idx (/ (length lst) 2))
        (return-from open-fridge-traj nil)))
    t))

;;
;; actions
;;
(warn ";; define grasp-can-single")
(defun grasp-can-single (obj &key (rotation 0))
  (let (via1 via2 tar orig-av)
    (send *ri* :stop-grasp :rarm)
    (send *tfb* :send-transform
		  (send (send *pr2* :worldcoords) :transformation (send obj :worldcoords))
          "/base_footprint" "/objecta_pose_frame")
    (setq orig-av (send *ri* :state :potentio-vector))

    (when *use-arm-navigation*
      (dotimes (i 5)
        (send *plan-env* :get-planning-scene)
        ;; (unix::usleep (* 400 1000))
        ))

    (setq tar (send (send (send *pr2* :worldcoords) :transformation
						  (send obj :worldcoords)) :copy-worldcoords))
    (send tar :orient 0 :z :world)

    (setq via1 (send tar :copy-worldcoords))
    (send via1 :translate (float-vector (* (cos (deg2rad rotation)) -100)
                                        (* (sin (deg2rad rotation))  100) 0) :world)

    ;; via2
    (setq via2 (send tar :copy-worldcoords))
    (send via2 :translate (float-vector 30 0 0) :world)

    ;; publish tf
    (send *tfb* :send-transform
          via1 "/base_footprint" "/objecta_pose_frame_via1")
    ;;(send *tfb* :send-transform
    ;;via2 "/base_footprint" "/objecta_pose_frame_via2")
    (send *tfb* :send-transform
          via2 "/base_footprint" "/objecta_pose_frame_via2")

    ;; move to via1
    (send *pr2* :rarm :inverse-kinematics 
		  (send via1 :transform (send *pr2* :worldcoords) :world)
		  :rotation-axis t)
    (send *tfb* :send-transform
          (send (send *pr2* :worldcoords) :transformation (send *pr2* :rarm :end-coords))
		  "/base_footprint" "/objecta_pose_end_coords")

    ;;(send *ri* :angle-vector (send *pr2* :angle-vector))
    ;;(send *ri* :wait-interpolation))
    ;; check IK
    (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...

    (cond
     ((null *use-arm-navigation*)
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (send *ri* :wait-interpolation))
     ((send *plan-env* :get-ik-for-pose (send *plan-env* :get-robot-coords :rarm) :rarm)
      (ros::ros-warn ";; can not solve collision free IK")
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (send *ri* :wait-interpolation))
     (t
      (when (send *plan-env* :planning-move-arm :rarm :planning-time 4.0)
        (ros::ros-warn ";; can not solve collision free Plan")
        (send *ri* :angle-vector (send *pr2* :angle-vector))
        (send *ri* :wait-interpolation))
      ))

    ;; move to via2 (grasping pose)
    (send *pr2* :rarm :inverse-kinematics
		  (send via2 :transform (send *pr2* :worldcoords) :world)
		  :rotation-axis t)
    ;; check IK
    #|(cond
    ((send *plan-env* :get-ik-for-pose (send *plan-env* :get-robot-coords :rarm) :rarm)
    (ros::ros-warn ";; can not solve collision free IK")
    ;; move to via1
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation))
    (t
    (when (send *plan-env* :planning-move-arm :rarm :planning-time 4.0)
    (ros::ros-warn ";; can not solve collision free Plan")
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation))
    ))|#
    (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;; grasp
    (send *ri* :start-grasp :rarm)

    ;; move to via1
    (send *pr2* :rarm :inverse-kinematics
		  (send via1 :transform (send *pr2* :worldcoords) :world)
		  :rotation-axis t)
    ;; check IK
    (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
    (cond
     ((null *use-arm-navigation*)
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (send *ri* :wait-interpolation))
     ((send *plan-env* :get-ik-for-pose (send *plan-env* :get-robot-coords :rarm) :rarm)
      (ros::ros-warn ";; can not solve collision free IK")
      ;; move to via1
      (send *ri* :angle-vector (send *pr2* :angle-vector))
      (send *ri* :wait-interpolation))
     (t
      (when (send *plan-env* :planning-move-arm :rarm :planning-time 4.0)
        (ros::ros-warn ";; can not solve collision free Plan")
        (send *ri* :angle-vector (send *pr2* :angle-vector))
        (send *ri* :wait-interpolation))
      ))
    ;;(send *ri* :angle-vector (send *pr2* :angle-vector))
    ;;(send *ri* :wait-interpolation)

    ;; move to original pose
    (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
    (send *ri* :angle-vector orig-av)
    (send *ri* :wait-interpolation)
    )
  )

(defun pr2-look-around (&key (time-tick 600) (show-wait 1200)
                             (yaw-angle-list (list 16 -16))
                             (pitch-angle-list (list 14 48))
                             (waist-z 140) (move-to-initial t))
  (let ((last-angle (send *pr2* :angle-vector))
        (y-angle yaw-angle-list)
        (p-angle pitch-angle-list))
    (send *pr2* :head :angle-vector (float-vector (car y-angle) (car p-angle)))
    (when move-to-initial
      (send *pr2* :rarm :angle-vector #f(-80.0 74.0 -20.0 -120.0 -20.0 -30.0 180.0))
      (send *pr2* :larm :angle-vector #f(80.0 74.0 20.0 -120.0 20.0 -30.0 180.0)))
    (send *pr2* :torso :angle-vector (float-vector waist-z))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1400)
    (send *ri* :wait-interpolation)

    (dolist (ya y-angle)
      (dolist (pa p-angle)
        (send *pr2* :head :angle-vector (float-vector ya pa))
        (send *ri* :angle-vector (send *pr2* :angle-vector) time-tick)
        (send *ri* :wait-interpolation)
        (unix::usleep (round (* show-wait 1000)))
        )
      (setq p-angle (nreverse p-angle)))

    (send *pr2* :angle-vector last-angle)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1400)
    (send *ri* :wait-interpolation)
    ))

(warn ";; define open-fridge-door")
(defvar *fridge-distance-threshold* 25.0)
(defvar *fridge-rotation-threshold* 0.09)
(defun open-fridge-door (&key (open-fridge-func #'open-fridge-traj)
                              (torso-lift 50) (head-pitch 0)
                              (door-type :circle) (look-around nil)) ;; :circle, :slide1, :slide2
  (defparameter *detection-topic* "/openni/rgb/ObjectDetection")
  (let (ret
        (idealcds
        (case door-type
          (:circle
           (make-coords :pos (float-vector 775 100 0)
                        ;;(float-vector 790 100 0) ;;(float-vector 760 100 0)
                        :rpy (float-vector 0.05 0 0)))
          (:slide1
           (make-coords :pos (float-vector 780 100 0)
                        :rpy (float-vector 0.0 0 0)))
          (:slide2
           (make-coords :pos (float-vector 750 100 0)
                        :rpy (float-vector 0.0 0 0)))))
        (diffcds (make-coords :pos (float-vector 10000 10000 10000)))
        cds)
    ;; initial pose
    (case door-type
      (:circle
       (send *pr2* :head :angle-vector (float-vector 0 head-pitch))
       (send *pr2* :torso :angle-vector (float-vector torso-lift))
       (send *ri* :angle-vector (send *pr2* :angle-vector))
       (send *ri* :wait-interpolation)
       )
      ((:slide1 :slide2)
       (send *pr2* :head :angle-vector (float-vector 0 head-pitch))
       (send *pr2* :torso :angle-vector (float-vector torso-lift))
       ;;(send *pr2* :larm :angle-vector (float-vector 25.0 74.0 35.0 -120.0 20.0 -30.0 0.0))
       (send *ri* :angle-vector (send *pr2* :angle-vector))
       (send *ri* :wait-interpolation)
       ))

    (dotimes (i 10 nil)
      ;; finding handle position
      (setq cds (check-detection :type "fridge" :speak-name *frige-speak-str*))

      ;; need detection fail check
      (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
      (when cds
        (send *obj* :reset-coords)
        (send *obj* :transform cds)
        (dump-structure *standard-output*
                        (list (send *pr2* :copy-worldcoords)
                              (send cds :copy-worldcoords)))
        (setq cds
              (make-coords :pos
                           (float-vector (elt (send cds :worldpos) 0)
                                         (elt (send cds :worldpos) 1) 0)
                           :rpy (list (elt (car (rpy-angle (send cds :worldrot))) 0)
                                      0 0)))

        (setq diffcds (send (send idealcds :inverse-transformation)
                            :transform (send cds :worldcoords) :world))

        (warn "~%~A -> ~A / ~A~%" diffcds cds idealcds)
        (ros::ros-info "DIFF: ~A" diffcds)
        (cond
         ((and (< (norm (float-vector (elt (send diffcds :worldpos) 0)
                                      (elt (send diffcds :worldpos) 1)))
                  *fridge-distance-threshold*)
               (< (abs (elt (car (rpy-angle (send diffcds :worldrot))) 0))
                  *fridge-rotation-threshold*))
          (setq cds (check-detection :type "fridge"
									 :speak-name *frige-speak-str*))
          (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
          ;; (y-or-n-p "Can I start open fridge? ")
          (when cds
            (if look-around (pr2-look-around))
            ;; for open fridge
            (case door-type
              (:circle
               (send *pr2* :head :angle-vector (float-vector 0 24))
               (send *pr2* :torso :angle-vector (float-vector 140))
               (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
               (send *ri* :wait-interpolation)
               )
              ((:slide1 :slide2)
               (send *pr2* :head :angle-vector (float-vector 0 50))
               (send *pr2* :torso :angle-vector (float-vector 140))
               (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
               (send *ri* :wait-interpolation)
               ))

            (setq cds
                  (make-coords :pos (send cds :worldpos)
                               :rpy (list (caar (send cds :rpy-angle)) 0 0)))
            (case door-type
              (:slide1
               (send cds :translate #f(0 -230 -355)))
              (:slide2
               (send cds :translate #f(0 -230 -675))))
            (setq *last-fridge-handle* cds)
			;; pr2 local coords transform to worldcoords
			(send cds :transform (send *pr2* :worldcoords) :world)
            (case door-type
              (:circle
               (setq ret (funcall open-fridge-func
                                  :rarm cds 1.7 ;;(/ pi 2)
                                  :rotation-axis t :radius 490
                                  :time-tick 280 ;; 600
                                  :wait-time 5.8 ;; t
                                  )))
              ((:slide1 :slide2)
               (setq ret (funcall open-fridge-func
                                  :rarm cds 320 ;; 330 ;; max
                                  :rotation-axis t :door-type :slide
                                  :time-tick 200 ;; 320
                                  :wait-time t
                                  )))
              )
            (if ret (return)))
          (if (and (boundp '*use-voicetext*) *use-voicetext*)
              (cond
               ((and (boundp '*use-english*) *use-english*)
                (speak-jp "I could not open the refrigerator."))
               (t (speak-jp "冷蔵庫を開けられませんでした。")))
            (speak-jp "れいぞうこを あけられません でした"))
          (send *pr2* :head :angle-vector (float-vector 0 head-pitch))
          (send *pr2* :torso :angle-vector (float-vector torso-lift))
          (send *ri* :angle-vector (send *pr2* :angle-vector))
          (send *ri* :go-pos-unsafe -0.1 0 0)
          (send *ri* :wait-interpolation)
          )
         (t ;;  there is much difference
          (send *ri* :go-pos-unsafe
                (/ (elt (send diffcds :worldpos) 0) 1100.0)
                (/ (elt (send diffcds :worldpos) 1) 1100.0)
                (* 0.9 (rad2deg (elt (car (rpy-angle (send diffcds :worldrot))) 0))))
          ;; wait ???
          (send *ri* :ros-wait 0.6 :spin-self t :spin t) ;; attention-check ...
          ;; (send *ri* :wait-interpolation)
          ))
        ))

    (when (not ret)
      (ros::ros-info "TIMEOUT: open-fridge-door");;
      (if (and (boundp '*use-voicetext*) *use-voicetext*)
          (cond
           ((and (boundp '*use-english*) *use-english*)
            (speak-jp "I could not find the refrigerator."))
           (t (speak-jp "冷蔵庫を見つけられませんでした。")))
        (speak-jp "れいぞうこ を みつけられません でした"))
      (send *ri* :ros-wait 2.0 :spin-self t :spin t) ;; attention-check ...
      (return-from open-fridge-door nil))

    (send *ri* :stop-grasp :rarm)
    (send *ri* :ros-wait 0.4 :spin-self t :spin t) ;; attention-check ...
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

    (case door-type
      (:circle
       ;; free door handle
       (let ((rend (send *pr2* :rarm :end-coords :copy-worldcoords))
             av1 av2)
         (send rend :translate (float-vector 30 -60 0) *pr2*)
         (send *pr2* :rarm :inverse-kinematics rend)
         (setq av1 (send *pr2* :angle-vector))
         (send rend :rotate pi/2 :x *pr2*)
         (send rend :translate (float-vector 0 -60 0) *pr2*)
         (send *pr2* :rarm :inverse-kinematics rend)
         (setq av2 (send *pr2* :angle-vector))
         (send *ri* :angle-vector-sequence
               (list av1 av2) (list 600 600))
         (send *ri* :wait-interpolation)
         )

       ;; ?? open door ()
       (send *ri* :angle-vector-sequence
             (list (float-vector 50 5 74 100 -90 -70 -6 20 -70 50 -59 -43 118 0 0 -2 31) ;; 1500
                   (float-vector 50 5 74 100 -90 -70 -6 20 -0 60 -20 -45 118 -90 0 -2 31) ;; 1000
                   (float-vector 50 5 74 100 -90 -70 -6 20 12 45 -34 -110 12 -38 74 -2 31) ;; 1000
                   (float-vector 50 5 74 100 -90 -70 -6 20 -54.745 34.2625 -34.6956 -83.6538 24.9161 -66.3615 118.904 -2.0 31.0) ;; 4500
                   (float-vector 50 5 74 100 -90 -70 -6 20 -54.745 34.2625 -34.6956 -83.6538 24.9161 -66.3615 118.904 -2.0 31.0) ;; 1500 ;; just wait
                   (float-vector 150 5 74 100 -90 -70 -6 20 -7 45 -34 -110 12 -38 74 -2 31)) ;; 2000
             (list 1500 1000 1000 4500 1500 2000))
       (send *ri* :wait-interpolation)

       (send *pr2* :angle-vector (float-vector 199.971 5.30455 72.1244 105.231 -88.5188 -69.9972 -5.72958 19.9717 31.3839 25.5029 23.0531 -118.916 160.305 -84.1469 160.058 -20 24))

       (send *ri* :angle-vector (send *pr2* :angle-vector))
       )
      ((:slide1 :slide2)
       (let ((rend (send *pr2* :rarm :end-coords :copy-worldcoords))
             avs)
         ;; should be changed
         (send rend :translate (float-vector -40 -160 0) *pr2*)
         (send *pr2* :inverse-kinematics rend
               :move-arm :rarm :revert-if-fail nil :use-torso 0.003)
         (push (send *pr2* :angle-vector) avs)
         (send rend :translate (float-vector 0 0 100) *pr2*)
         (send *pr2* :inverse-kinematics rend
               :move-arm :rarm :revert-if-fail nil :use-torso 0.002)
         (push (send *pr2* :angle-vector) avs)
         (send *ri* :angle-vector-sequence (nreverse avs) (list 800 800))
         (send *ri* :wait-interpolation)
         (send *pr2* :rarm :angle-vector #f(-45.0 55.0 -15.0 -120.0 12.0 -38.0 74.0))
         (send *ri* :angle-vector (send *pr2* :angle-vector) 1600)
         ))
      )
    (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
    (send *ri* :wait-interpolation)
    t
    ))

(warn ";; define grasp-can")
(defvar *use-arm-navigation* nil)
(defun grasp-can (&key (move t) (rotation))
  ;; move-to spot for grasping
  (when move
    (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
    (ros::ros-info "GO POS UNSAFE for grasping can")
    (send *ri* :go-pos-unsafe 0 0 (rad2deg 0.38))
    (send *ri* :go-pos-unsafe 0.185 -0.25 0) ;; 
    (send *ri* :wait-interpolation))

  (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

  (let (isgrasp)
    (dotimes (trial 10)
      ;; detect cans which was indicated by ( type )
      ;;(when (not (setq cds (check-detection :type *type* :single t)))
      ;;(setq cds (check-detection :type *type* :tf-force t :timeout 30 :single t)))
      (let (cds)
        (unless (setq cds (check-detection :speak-name *type*
                           :type *type* :tf-force t :timeout 35 :single t))
          (setq cds (check-detection :speak-name *type*
                     :type *type* :tf-force t :timeout 70 :single t)))
        (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
        (when cds
          ;; (speak-jp (format nil "~A を とりだします" *type*))
          (send *obj* :reset-coords)
          (send *obj* :transform cds)
          (warn ";; print check cans~%")
          (dump-structure *standard-output*
                          (list (send *pr2* :copy-worldcoords)
                                (send cds :copy-worldcoords)))
          (unless (boundp '*tfb*)
            (setq *tfb* (instance ros::transform-broadcaster :init)))
          (send *tfb* :send-transform
                cds "/base_footprint" "/object_pose_frame")
          (send *ri* :ros-wait 0.0 :spin-self t :spin t) ;; attention-check ...
          (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
          (unless rotation
            ;; FIXME: calc rotation from /map and /base_footprint
            (setq rotation 0))
		  (send cds :transform (send *pr2* :worldcoords) :world)
          (grasp-can-single cds :rotation rotation)
          (setq isgrasp (< 10 (send *ri* :start-grasp :rarm)))
          (when isgrasp
            (when move (send *ri* :go-pos-unsafe -0.3 0.05 -20))
            (send *ri* :ros-wait 1.0 :spin-self t :spin t) ;; attention-check ...
            (return-from grasp-can t))
          ;; (unix::sleep 2)
          (if (and (boundp '*use-voicetext*) *use-voicetext*)
              (cond
               ((and (boundp '*use-english*) *use-english*)
                (speak-jp "I failed to pick up. I will retry."))
               (t (speak-jp "失敗しましたもう一度やってみます。")))
            (speak-jp "しっぱいしました  もういちど やって みます"));;
          )
        )))
  (if (and (boundp '*use-voicetext*) *use-voicetext*)
      (cond
       ((and (boundp '*use-english*) *use-english*)
        (speak-jp (format nil "I could no pick up ~A" *type*)))
       (t (speak-jp (format nil "~Aを取り出しませんでした。" *type*))))
    (speak-jp (format nil "~A を とりだし ません でした" *type*)))
  (when move (send *ri* :go-pos-unsafe -0.3 0.05 -20))
  (send *ri* :ros-wait 1.0 :spin-self t :spin t) ;; attention-check ...
  nil)

(warn ";; define close-fridge")
(defun close-fridge ()
  ;; close the door of fridge
  (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
  (send *ri* :angle-vector-sequence
        (list (float-vector 250 5 74 100 -90 -70 -6 20 -13 50 -30 -25 118 -90 62 -2 31);; 2000
              (float-vector 200 5 74 100 -90 -70 -6 20 -50 54 -15 -43 118 -90 62 -2 31);; 1000
              (float-vector 150 5 74 100 -90 -70 -6 20 -80 10 -39 -33 -49 -32 89 -2 32);; 1500
              (float-vector 100 5 74 100 -90 -70 -6 20 -61 6 -74 -44 55 -75 102 -2 33);; 2500
              ;;(float-vector 50 5 74 100 -90 -70 -6 20 9.05631 26.4211 -42.0907 -111.215 97.6198 -19.5577 -24.0422 -2.0 33.0)
              (float-vector 75.0 5.0 74.0 100.0 -90.0 -70.0 -6.0 20.0 -25.5 9.00495 -70.377 -55.5259 61.1911 -62.0514 66.163 -2.0 33.0) ;; 2000
              (float-vector 50 5 74 100 -90 -70 -6 20 10 12.0099 -66.7541 -67.0519 67.3822 -49.1028 30.3261 -2.0 33.0))  ;; 1500
        (list 2000 1000 1500 1500 2000 1500))
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :ros-wait 0.0 :spin-self t) ;; attention-check ...
  (pr2-tuckarm-pose :rarm)
  t
  )

(defun put-can-on-turtlebot ()
  (send *ri* :update-robot-state)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (pr2-tuckarm-pose :rarm)
  (send *pr2* :rarm :inverse-kinematics (make-coords :pos (float-vector 800 0 730)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
  (send *ri* :wait-interpolation)
  (send *pr2* :rarm :inverse-kinematics (make-coords :pos (float-vector 800 0 530)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :rarm 0.08 :wait t)
  (send *pr2* :rarm :inverse-kinematics (make-coords :pos (float-vector 650 0 530)))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :rarm 0.005 :wait t)
  (pr2-tuckarm-pose :rarm)
  )

(defun put-can-on-turtlebot2 ()
  (send *ri* :update-robot-state)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (pr2-tuckarm-pose :rarm)
  (send *pr2* :rarm :inverse-kinematics (send (make-coords :pos (float-vector 800 0 730)) :rotate (deg2rad 180) :x))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
  (send *ri* :wait-interpolation)
  (send *pr2* :rarm :inverse-kinematics (send (make-coords :pos (float-vector 800 0 550)) :rotate (deg2rad 180) :x))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :rarm 0.08 :wait t)
  (send *pr2* :rarm :inverse-kinematics (send (make-coords :pos (float-vector 650 0 530)) :rotate (deg2rad 180) :x))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :rarm 0.005 :wait t)
  (pr2-tuckarm-pose :rarm)
  )


(defun distance-of-front-cb (msg)
  (let* ((pc (send msg :ranges))
		 (about-front (remove-if #'zerop 
								 (subseq pc (- (/ (length pc) 2) 10) (+ (/ (length pc) 2) 10)))))
	(push (/ (reduce #'+ about-front) (length about-front))
		  *averages*)
	))

(defun distance-of-front ()
  (ros::roseus "place_tray_base_scan_calc")
  (ros::roseus-add-msgs "sensor_msgs")
  (ros::roseus-add-msgs "posedetection_msgs")
  (ros::roseus-add-msgs "geometry_msgs")
  (defvar *base-scan-id* "/base_scan")
  (defparameter *averages* nil)

  (ros::subscribe *base-scan-id*
				  sensor_msgs::LaserScan
				  #'distance-of-front-cb)

  (ros::rate 50)
  (when (ros::ok)
	(dotimes (i 30)
	  (ros::spin-once)
	  (ros::sleep)
	  ))
  (ros::unsubscribe *base-scan-id*)
  (/ (reduce #'+ *averages*) (length *averages*))
  )

(warn ";; define place-tray")
(defun place-tray (&key (table-height 850.0)
                        (up-height-tolerance 150.0)
                        (down-height-tolerance -20.0)
                        (use-base-scan nil)
                        (go-forward 0)
                        (forward-dist 450.0))
    ;;
    ;; place_tray function
    ;;
    ;; Before this function is called, the robot is assumed to have the tray gripped in both grippers
    ;; and the tray is posiitoned directly above the table.
    ;;
  (let (arm-mid-pos
        arm-mid-pos-pr2)

    ;; get current gripper pose
    (send *pr2* :torso :waist-z :joint-angle 250)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    (setq arm-mid-pos
          (apply #'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos)))

    (setq arm-mid-pos-pr2 (send (send *pr2* :worldcoords) :inverse-transform-vector
                                (apply #'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos))))
    ;;raise tray
    (let ((av-seq nil))
      (send *pr2* :arms :move-end-pos
            (float-vector 0 0 (- (+ table-height up-height-tolerance) (elt arm-mid-pos 2))) :world)
      (push (send *pr2* :angle-vector) av-seq)

      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      ;; put forward a tray
      (send *pr2* :arms :move-end-pos
            (send *pr2* :rotate-vector (float-vector (- forward-dist (elt arm-mid-pos-pr2 0)) 0 0)) :world)
      (push (send *pr2* :angle-vector) av-seq)
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

      (setq av-seq (reverse av-seq))
      (send *ri* :angle-vector-sequence av-seq (list 2000 2000))
      (send *ri* :wait-interpolation)
      )

    ;; for navigation
    (when use-base-scan
      (setq go-forward (* (- (distance-of-front) 0.1) 1000))
      (if (> 0.1 go-forward) (setq go-forward 0)))
    (when (> go-forward 10)
      (speak-jp "まえにすすみます")
      (send *pr2* :translate (float-vector go-forward 0 0))
      (send *ri* :go-pos-unsafe (/ go-forward 1000.0) 0)
      )

    ;; put down a tray
    (send *pr2* :arms :move-end-pos
          (float-vector 0 0 (- (- table-height down-height-tolerance) (elt arm-mid-pos 2))) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    ;; open grippers
    (send *ri* :stop-grasp :arms :wait t)
    ;; (send *pr2* :larm :end-coords :dissoc *tray*) ;; ???

    ;; move arms apart
    (send *pr2* :arms :move-end-pos (float-vector 0 0 100) :world)
    (send *pr2* :arms :move-end-pos (float-vector 0 0 100) :world)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    ;; for navigation
    (when (> go-forward 10)
      (speak-jp "うしろにさがります")
      (send *pr2* :translate (float-vector (- go-forward) 0 0))
      (send *ri* :go-pos-unsafe (/ go-forward -1000.0) 0)
      )

    ;; revert torso link
    (send *pr2* :torso :waist-z :joint-angle 50)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    ;;dont wait interpolation
    (pr2-reset-pose)
    t
    ))

(warn ";; define pick-tray")
(defun pick-tray (&key (target *tray*) (manip-pose t))
  (let (tray-larm-handle tray-rarm-handle
        (grasp-threshold (list 3 3))
        (grasp-args)) ;; use let to define local variables
    (warn "pick-tray-table-side ~%")
	(load "package://jsk_2013_10_pr2_73b2/euslisp/objectdetection.l")
	(defparameter *room73b2-origin* "/eng2/7f/73B2")
    (when manip-pose
      (pr2-pick-tray-pose :reset-pose t)
      (send *pr2* :head :neck-p :joint-angle 50)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      )

    ;;detect tray when real robot
    (if (send *ri* :joint-action-enable)
        (let ((cnt 0))
          (while (and (<= cnt 3) (not (tray-detection *tray*)))
            (send *pr2* :head :neck-p :joint-angle (+ 50 (* (10 (incf cnt)))))
            (send *ri* :angle-vector (send *pr2* :angle-vector) 100)
            (send *ri* :wait-interpolation)
            ))
      )

    (setq tray-larm-handle
          (send (send (send *tray* :handle-larm-handle) :copy-worldcoords)
                :rotate (deg2rad -20) :z)) ;; do not move in world coords, use object(tray) relative coords
    (setq tray-rarm-handle
          (send (send (send *tray* :handle-rarm-handle) :copy-worldcoords)
                :rotate (deg2rad  20) :z)) ;; need copy-worldcoords, otherwise move handle coords directory

    ;; Open gripper
    (send *ri* :stop-grasp :arms) ;; stop-grasp wait until gripper motion stops
    ;; wait interpolation

    ;; use dual arm IK (see https://sourceforge.net/p/jsk-ros-pkg/code/4103/)
    (if (boundp '*irtviewer*)
        (send-all (list tray-larm-handle tray-rarm-handle) :draw-on :flush t)) ;; use :draw-on to confirm coords position

    (let (avs)
      (dolist (offset (list (cons #f(-100 -10 0) #f(-100 10 0))
                            (cons #f(0 30 0) #f(0 -30 0))))
        (send *pr2* :inverse-kinematics
              (list   ;; use local coords, this cod works if tray is placed on different orientation
               (send (send tray-larm-handle :copy-worldcoords) :translate (car offset))
               (send (send tray-rarm-handle :copy-worldcoords) :translate (cdr offset)))
              :use-torso t
              :look-at-target (midpoint 0.5
                                        (send tray-larm-handle :worldpos)
                                        (send tray-rarm-handle :worldpos)))
        (push (send *pr2* :angle-vector) avs)
        )
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      ;;(print (send *pr2* :angle-vector))
      ;;(print (send *pr2* :arms :wrist-r :joint-angle))
      (send *ri* :angle-vector-sequence (reverse avs) (list 1500 700))
      ;;(send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 1000 :arms)
      (send *ri* :wait-interpolation)
      )

    (warning-message 3 "grasp tray~%")
    ;; Grasp it
    (setq grasp-args (send *ri* :start-grasp :arms :gain 1 :objects
                           (list (find *tray* (send *ri* :objects)
                                       :test #'(lambda (a b) (string= (send a :name) (send b :name)))))))
    ;; stop-grasp wait until gripper motion stops

    ;; check if tray is grasped
    (warning-message 3 "check tray ~A > ~A~%" grasp-args grasp-threshold)
    (when (and (send *ri* :joint-action-enable)
               (or (< (elt grasp-args 0) (elt grasp-threshold 0))
                   (< (elt grasp-args 1) (elt grasp-threshold 1))))
      (warning-message 3 "tray is not grasped~%")
      (speak-jp "can not grasp tray")

      ;; Open gripper
      (send *ri* :stop-grasp :arms) ;; stop-grasp wait until gripper motion stops
      (pr2-pick-tray-pose)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (return-from pick-tray nil)
      )
    (send *pr2* :larm :end-coords :assoc *tray*) ;; ???

    (let (avs)
      ;; Hold it up!
      (send *pr2* :arms :move-end-pos #f(0 0 100) :world)
      (send *pr2* :look-at-hand :arms)
      (push (send *pr2* :angle-vector) avs)

      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

      ;; Move it close to PR2
      (send *pr2* :arms :move-end-pos #f(0 0 -250))
      (send *pr2* :look-at-hand :arms)
      (push (send *pr2* :angle-vector) avs)

      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

      (send *ri* :angle-vector-sequence (reverse avs) 1000) ;; use angle-vector-sequence
      (send *ri* :wait-interpolation)
      )

    ;; Go back 50cm
;;    (send *ri* :go-pos-unsafe -0.4 0 90)
    t))

(ros::roseus-add-msgs "pr2_gripper_sensor_msgs")
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

(provide :pr2-action)