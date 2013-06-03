#!/usr/bin/env roseus

(ros::roseus-add-msgs "posedetection_msgs")
(load "models/arrow-object.l")

(defvar *base-frame-id* "/base_footprint")
(defvar *pub-laundry-button-topic* "/laundry_button_marker")

(load "package://jsk_demo_common/euslisp/pr2-action.l")
(load "package://jsk_2013_04_pr2_610/euslisp/objectdetection-laundry.l")

(ros::roseus "laundry_button_marker_publisher")
(ros::advertise *pub-laundry-button-topic* visualization_msgs::Marker 5)

(defun put-cloth-into-laundry ()
  (warn "put-cloth-into-laundry is too long. So ...~%")
  (warn "use (put-cloth)~%")
)

(defun put-cloth ()
  (pr2-reset-pose)  ;; debug
  (send *pr2* :larm :collar-y :joint-angle 90)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (open-door)
  (put-cloth-into)
  (send *pr2* :larm :collar-y :joint-angle -90)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (close-door)
  (pr2-reset-pose)
)

(defun open-door ()
  (warn "open-door ~%")
  (let (button)


    (send *ri* :start-grasp :rarm)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :head :neck-y :joint-angle -4.353)
    (send *pr2* :head :neck-p :joint-angle 24.422)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;; change laundry button pos according to image processing
    (send *laundry* :move-to (laundry-detection *laundry*) :world);; FIX ME returns t if found, nil not found
	(if (boundp '*irtviewer*) (send (send *laundry* :copy-worldcoords) :draw-on :flush t :size 1700 :color #f(0 1 0)))
    (warn "laundry moved to ~A~%" (send *laundry* :coords))
    (setq button (send (make-coords :pos (float-vector 770 112 780)
				    :rpy (list (deg2rad 180) (deg2rad 30) (deg2rad 0)))
		       :transform (send *laundry* :copy-worldcoords) :world))
	;; debug
	(if (boundp '*irtviewer*) (send button :draw-on :flush t :size 1000))
    (warn "button: ~A~%" button)

    ;; publish button marker
    (ros::publish *pub-laundry-button-topic* 
		  (coords->marker-msg
		   button
		   (instance std_msgs::header :init
			     :stamp (ros::time-now)
			     :frame_id *base-frame-id*)
		   :ns "laundry_button"
		   :lifetime 300))

    ;; move right hand out
;    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
;	(pr2-reset-pose)
;    (send *pr2* :rarm :collar-y :joint-angle -90)
;   (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
;   (send *ri* :wait-interpolation)
;	(send *pr2* :rarm :inverse-kinematics
;		  (send (make-coords :pos (float-vector 820 120 780))
;				:transform (send *laundry* :copy-worldcoords) :world)
;		  :rotation-axis t
;		  :look-at-target t
;		  :revert-if-fail t
;		  :debug-view nil)
;    (send *pr2* :rarm :move-end-pos #f(200 200 200) :world)
;    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
;    (send *ri* :wait-interpolation)

    ;; ready to push
    (send *pr2* :rarm :inverse-kinematics
	  button
	  :rotation-axis t
	  :look-at-target t
	  :debug-view nil
	  :revert-if-fail t)
    (send *pr2* :rarm :move-end-pos #f(-40 0 0))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
;    (send *pr2* :rarm :wrist-r :joint-angle 90)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :wait-interpolation)

    ;; push
    (send *pr2* :rarm :move-end-pos (float-vector 150 0 0))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)

;	(do ((i 180 (+ i 45)) (r 2))
;		((> i 540) t)
;	  (progn 
		(send *pr2* :rarm :move-end-pos (float-vector -50 -5 0) (send *pr2* :worldcoords))
		(send *ri* :angle-vector (send *pr2* :angle-vector) 500)
		(send *ri* :wait-interpolation)
		(send *pr2* :rarm :move-end-pos (float-vector 50 0 0) (send *pr2* :worldcoords))
		(send *ri* :angle-vector (send *pr2* :angle-vector) 500)
		(send *ri* :wait-interpolation)
		(send *pr2* :rarm :move-end-pos (float-vector -50 0 -5) (send *pr2* :worldcoords))
		(send *ri* :angle-vector (send *pr2* :angle-vector) 500)
		(send *ri* :wait-interpolation)
		(send *pr2* :rarm :move-end-pos (float-vector 50 0 0) (send *pr2* :worldcoords))
		(send *ri* :angle-vector (send *pr2* :angle-vector) 500)
		(send *ri* :wait-interpolation)
;		)
;	  )

    ;; pull arm for preventing collision
    (send *pr2* :rarm :move-end-pos #f(-200 0 0))
    (send *pr2* :rarm :move-end-pos #f(-50 0 -150) (send *pr2* :worldcoords))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation)
    )
  )

(defun put-cloth-into ()
  (warn "put-cloth~%")
  (let (standby release)
    (setq standby (send (make-coords :pos #f(800 -110 680)
				     :rpy (list (deg2rad 180) (deg2rad 0) (deg2rad 30)))
			:transform *laundry* :world))
    (warn "release: ~A~%" release)
    (send *pr2* :rarm :inverse-kinematics
	  standby
	  :rotation-axis t
	  :look-at-target t
	  :debug-view nil
	  :revert-if-fail t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (setq release (send (make-coords :pos #f(650 -110 480)
				     :rpy (list (deg2rad 180) (deg2rad 30) (deg2rad 0)))
			:transform *laundry* :world))
    (warn "release: ~A~%" release)

    (send *pr2* :rarm :inverse-kinematics
	  release
	  :rotation-axis t
	  :look-at-target t
	  :debug-view nil
	  :revert-if-fail nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    ;; release cloth
    (send *ri* :stop-grasp :rarm :wait nil)
    (send *pr2* :rarm :wrist-r :joint-angle -180)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *pr2* :rarm :wrist-r :joint-angle 0)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation)


    ;; move back hand
    (send *pr2* :rarm :inverse-kinematics
	  standby
	  :rotation-axis t
	  :look-at-target t
	  :debug-view nil
	  :revert-if-fail t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
    (send *ri* :wait-interpolation)

    (send *pr2* :rarm :move-end-pos #f(-100 0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)

    (send *pr2* :rarm :move-end-pos #f(-70 -500 0) (send *pr2* :worldcoords))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    )
  )

(defun close-door ()
  (warn "close-door~%")
  (let ((handle (send *laundry* :handle-handle-cap))
	 release)

    ;; ready to close
    (send *pr2* :larm :move-end-pos (float-vector 50 -120 0) (send *pr2* :worldcoords))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "ready to close")

    ;; close the cap
    (setq release (send (make-coords :pos #f(650 -110 530)
				     :rpy (list (deg2rad 180) (deg2rad 0) (deg2rad 0)))
			:transform *laundry* :world))
    (warn "release: ~A~%" release)

    (send *pr2* :larm :inverse-kinematics
	  release
	  :rotation-axis t
	  :look-at-target t
	  :debug-view nil
	  :revert-if-fail nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation)
    )
  )


(defun put-cloth-old ()
  (warn "open-laundry-door (kuroiwa)~%")
  (let* ((release (send *laundry* :handle-contents))
         (handle (send *laundry* :handle-handle-cap))
         (button (send *laundry* :handle-open-button))
         (button-detected button)
         (debug nil)
         (marker-coords (check-detection :type "laundry_center" :speak-name "せんたくき" :target-object *laundry*))
         (r 500) (depth 500) (near 80) (cap-point (make-coords)) dst front)

    ;; debug
    (pr2-reset-pose)

    ;; change laundry button pos according to image processing
    (send *ri* :start-grasp :rarm)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :head :neck-y :joint-angle -4.353)
    (send *pr2* :head :neck-p :joint-angle 24.422)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    (laundry-detection *laundry*)
    (setq release (send *laundry* :handle-contents))
    (setq handle (send *laundry* :handle-handle-cap))
    (setq button (send (make-coords :pos #f(770 120 780)
				    :rpy (list (deg2rad 180) (deg2rad 30) (deg2rad 0)))
		       :transform *laundry* :world))
    (print button)
    ;; publish button marker
    (ros::publish *pub-laundry-button-topic* 
		  (coords->marker-msg
		   button
		   (instance std_msgs::header :init
			     :stamp (ros::time-now)
			     :frame_id *base-frame-id*)
		   :ns "laundry_button"
		   :lifetime 300))    
;    (send *pr2* :rarm :inverse-kinematics
;	  (send button :worldcoords))
;    (send *ri* :angle-vector (send *pr2* :angle-vector))
;    (send *ri* :wait-interpolation)
	  
;    (return-from put-cloth nil)

    (when marker-coords
      (warn "laundry position changed by image processing to ~A~%" marker-coords))


    ;;STEP0 open collar-y
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :rarm :collar-y :joint-angle -90)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    (send *pr2* :rarm :move-end-pos #f(200 200 200) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;; STEP1 Push the Button
#|    (setq dst (make-coords)
	  front 120)
    (send dst :init
          :pos (send (send (send button :copy-worldcoords) :translate (float-vector (- (* near 1.2)) -30 25) (send *pr2* :worldcoords)) :worldpos)
          :rot (send (send (send *pr2* :copy-worldcoords) :rotate (deg2rad -45) :x) :worldrot))

    (send dst :rotate (deg2rad -135) :y :world)
|#

    ;; ready to push
    (send *pr2* :rarm :inverse-kinematics
          button
	  :rotation-axis t
          :look-at-target t
          :debug-view debug
          :revert-if-fail nil)
    (send *pr2* :rarm :move-end-pos #f(-50 0 0))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *pr2* :rarm :wrist-r :joint-angle 90)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "ready to put")

    ;; push
    (send *pr2* :rarm :move-end-pos (float-vector 150 0 0))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 100)
    (send *ri* :wait-interpolation)
    (print "push")

    ;; pull the right arm for opening the cap
;    (send *pr2* :rarm :move-end-pos (float-vector -150 0 -30))
    (send *pr2* :rarm :move-end-pos (float-vector -200 0 -30))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) )
    (send *ri* :wait-interpolation)
    (print "pull")


;    (send *pr2* :rarm :move-end-pos (float-vector 0 -80 0))
    (send *pr2* :rarm :move-end-pos (float-vector 0 -120 0) :world)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "pull")


    ;; store current position of right arm
    (setq former (send (send *pr2* :rarm :end-coords) :copy-worldcoords))

;    (speak-en "STEP1 complete")
    (print "STEP1 done")

    ;; STEP2 Open the Cap
    (setq dst (make-coords))
    ;; modify the model's parameters
    (send cap-point :init 
          :pos (send 
                (send (send handle :copy-worldcoords) :translate #f(-60 80 0)
                      (send *pr2* :worldcoords))
                :worldpos)
          :rot (send handle :worldrot))

    ;; insert the right arm into the gap
    (send *pr2* :rarm :inverse-kinematics
          cap-point 
          :look-at-target t
          :debug-view debug
          :revert-if-fail nil)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "insert")
    
    ;; open the cap
    ;; modify the model's parameters
    (send dst :init 
          :pos (send 
                (send (send release :copy-worldcoords) :translate
                      #f(130 -50 90) (send *pr2* :worldcoords))
                :worldpos)
          :rot (send *pr2* :worldrot))

    ;; ready to put
    (send *pr2* :rarm :inverse-kinematics
          (send (send dst :copy-worldcoords) 
                :translate (float-vector (- (* depth 0.4)) 0 near) (send *pr2* :worldcoords))
          :look-at-target t
          :debug-view debug
          :revert-if-fail nil)

    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "ready to put")

    ;; put the cloth
    (send *pr2* :rarm :inverse-kinematics
          dst
          :rotation-axis nil
          :look-at-target t
          :debug-view debug
          :revert-if-fail nil)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp :rarm)
    (send *pr2* :rarm :wrist-r :joint-angle -180)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (send *pr2* :rarm :wrist-r :joint-angle 0)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    ;; (send *ri* :start-grasp :rarm)
    (print "put the cloth")

    ;; pull the right arm
    (send *pr2* :rarm :inverse-kinematics
          (send (send dst :copy-worldcoords)
                :translate (float-vector (- (* depth 0.7)) 0 (* near 1.2)) (send *pr2* :worldcoords))
          :look-at-target t
          :debug-view debug
          :revert-if-fail nil)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "pull the right arm")


;    (speak-en "STEP3 complete")

    ;; STEP4 Close the Cap

    ;; move the right arm in order to avoid the cap
    (send *pr2* :rarm :inverse-kinematics
          (send (send former :copy-worldcoords)
                :translate #f(-100 -200 -100) (send *pr2* :worldcoords))
          :rotation-axis nil
          :debug-view debug
          :revert-if-fail nil)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "move the right arm")

    ;; ready to close
    (send *pr2* :larm :move-end-pos (float-vector 0 (- (* r 0.6)) 0) (send *pr2* :worldcoords))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "ready to close")

    ;; close the cap
    (send *pr2* :larm :inverse-kinematics
          (send (send cap-point :copy-worldcoords) 
                :translate (float-vector (+ near 50) 0 0) (send *pr2* :worldcoords))
          :rotation-axis nil
          :debug-view debug
          :revert-if-fail nil)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (print "close the cap")


;    (speak-en "STEP4 complete")

    ;; initialize the pose
    (send *pr2* :reset-pose)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
;    (speak-jp "ミッションコンプリート")
    ))
