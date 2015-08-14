#!/usr/bin/env roseus


(ros::roseus-add-msgs "posedetection_msgs")
(load "models/arrow-object.l")

(defvar *base-frame-id* "/base_footprint")
(defvar *pub-laundry-button-topic* "/laundry_button_marker")

(load "package://jsk_demo_common/euslisp/pr2-action.l")
(load "package://jsk_2013_04_pr2_610/euslisp/objectdetection.l")

(ros::roseus "laundry_button_marker_publisher")
(ros::advertise *pub-laundry-button-topic* visualization_msgs::Marker 5)

(defun put-cloth-into-laundry ()
  (warn "put-cloth-into-laundry is too long. So ...~%")
  (warn "use (put-cloth)~%")
)

(defun put-cloth-old ()
  (send *pr2* :larm :collar-y :joint-angle 90)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (open-laundry)
  (put-cloth-into)
  (send *pr2* :larm :collar-y :joint-angle -90)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (close-laundry)
  (pr2-reset-pose)
  t
)

(defun put-cloth-larm ()
;  (send *pr2* :larm :collar-y :joint-angle 110)
;  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (open-laundry)
;  (send *ri* :go-pos-unsafe -0.5 0)
;  (pass-cloth-to-rarm)
;  (send *ri* :go-pos-unsafe 0.5 0)
  (put-cloth-laundry)
;  (send *pr2* :larm :collar-y :joint-angle -90)
;  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (close-laundry)
  (push-button-laundry)
  (pr2-reset-pose)
  t
)

(defun pass-cloth-to-rarm ()
  (warn "pass-cloth-to-rarm ~%")
  (send *ri* :go-pos-unsafe -0.5 0)

  (let (pass-coords pass-coords-r pass-coords-l)
    (setq pass-coords (send (send *pr2* :copy-worldcoords) :translate #f(400 0 800)))
    (setq pass-coords-r (send (send (send pass-coords :copy-worldcoords) :translate #f(0 0 -70)) :rotate pi/2 :z))
    (setq pass-coords-l (send (send pass-coords :copy-worldcoords) :rotate -pi/2 :z))
    
    (send *pr2* :reset-pose)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :inverse-kinematics
	  pass-coords-l
	  :rotation-axis t
          :look-at-target t
	  :debug-view nil
	  :revert-if-fail t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    (send *ri* :stop-grasp :rarm :wait t)
    
    (let (av-seq)
      (send *pr2* :rarm :inverse-kinematics
	    (send (send pass-coords-r :copy-worldcoords) :translate #f(-100 0 0))
            :look-at-target t
	    :rotation-axis t
	    :debug-view nil
	    :revert-if-fail t)
      (push (send *pr2* :angle-vector) av-seq)
      ;;      (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      ;;    (send *ri* :wait-interpolation)
      (send *pr2* :rarm :inverse-kinematics
	    pass-coords-r
            :look-at-target t
	    :rotation-axis t
	    :debug-view nil
	    :revert-if-fail t)
      (push (send *pr2* :angle-vector) av-seq)
      (setq av-seq (reverse av-seq))
      (send *ri* :angle-vector-sequence av-seq (list 2000 1000))
      ;;(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      )

    (send *ri* :start-grasp :rarm)
    (send *ri* :stop-grasp :larm :wait t)

    (pr2-reset-pose)
    (send *pr2* :larm :collar-y :joint-angle 110)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    )
  t
  )


(defun open-laundry ()
  (warn "open-laundry ~%")
  (send *pr2* :larm :collar-y :joint-angle 110)
  (send *ri* :angle-vector (send *pr2* :angle-vector))

  (let (button)
    (send *ri* :start-grasp :rarm)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :head :neck-y :joint-angle -4.353)
    (send *pr2* :head :neck-p :joint-angle 24.422)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)

    ;; change laundry button pos according to image processing
    (if (not (send *ri* :simulation-modep))
	(send *laundry* :move-to (laundry-detection *laundry*) :world);; FIX ME returns t if found, nil not found
    )
    (if (boundp '*irtviewer*) (send (send *laundry* :copy-worldcoords)
                                    :draw-on :flush t
                                    :size 1700 :color #f(0 1 0)))
    (warn "laundry moved to ~A~%" (send *laundry* :coords))
    (setq button (send (make-coords :pos (float-vector 770 112 790)
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

    (let (av-seq)
      (setq av-seq nil)
      ;; ready to push
      (send *pr2* :rarm :inverse-kinematics
	    button
	    :rotation-axis t
	    :look-at-target t
	    :debug-view nil
	    :revert-if-fail t)
      (send *pr2* :rarm :move-end-pos #f(-40 0 0))
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (push (send *pr2* :angle-vector) av-seq)
      
      ;; push
      (send *pr2* :rarm :move-end-pos (float-vector 30 0 0))
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (push (send *pr2* :angle-vector) av-seq)

      (send *pr2* :rarm :move-end-pos (float-vector 100 0 0))
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (push (send *pr2* :angle-vector) av-seq)
      
      (setq av-seq (reverse av-seq))
      (send *ri* :angle-vector-sequence av-seq (list 4000 2000 1000))
      (send *ri* :wait-interpolation)

      (setq av-seq nil)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

      (send *pr2* :rarm :move-end-pos (float-vector -50 0 0))
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (push (send *pr2* :angle-vector) av-seq)

      (send *pr2* :rarm :move-end-pos #f(0 50 0) (send *pr2* :worldcoords))
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (push (send *pr2* :angle-vector) av-seq)
      (print (send *pr2* :angle-vector))

      ;; break open door
      (send *pr2* :rarm :inverse-kinematics
	    (make-coords :pos (send (send *pr2* :rarm :end-coords) :worldpos)
			 :rpy (list (deg2rad 140) 0 0))
	    :rotation-axis t
	    :debug-view nil)
      (send *pr2* :rarm :move-end-pos #f(0 450 0) (send *pr2* :worldcoords))
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (push (send *pr2* :angle-vector) av-seq)

      (send *pr2* :rarm :move-end-pos #f(-200 0 0) (send *pr2* :worldcoords) :rotation-axis :z)
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (push (send *pr2* :angle-vector) av-seq)
      
      (send *pr2* :rarm :move-end-pos #f(0 0 -200) (send *pr2* :worldcoords) :rotation-axis :z)
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
      (push (send *pr2* :angle-vector) av-seq)

      (setq av-seq (reverse av-seq))
      (send *ri* :angle-vector-sequence av-seq (list 1000 2000 3000 500 1500))
      (send *ri* :wait-interpolation)
      )
    )
  t
  )

(defun put-cloth-laundry ()
  (warn "put-cloth-laundry~%")
  (pass-cloth-to-rarm)
  (send *ri* :go-pos-unsafe 0.5 0)

  (let (standby release av-seq)
    (setq av-seq nil)
    (setq standby (send (make-coords :pos (float-vector 800 -110 680)
				     :rpy (list (deg2rad 180) (deg2rad 0) (deg2rad 30)))
			:transform *laundry* :world))
    (warn "release: ~A~%" release)
    (send *pr2* :rarm :inverse-kinematics
	  standby
	  :rotation-axis t
	  :look-at-target t
	  :debug-view nil
	  :revert-if-fail t)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (push (send *pr2* :angle-vector) av-seq)
    ;;(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    ;;(send *ri* :wait-interpolation)
    (setq release (send (make-coords :pos (float-vector 650 -110 480)
				     :rpy (list (deg2rad 180) (deg2rad 30) (deg2rad 0)))
			:transform *laundry* :world))
    (warn "release: ~A~%" release)

    (send *pr2* :rarm :move-end-pos #f(200 0 -100) (send *pr2* :worldcoords) :rotation-axis t)
    (push (send *pr2* :angle-vector) av-seq)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    ;;(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    ;;(send *ri* :wait-interpolation)
    (send *pr2* :rarm :move-end-pos #f(100 0 0) (send *pr2* :worldcoords) :rotation-axis t)
    (push (send *pr2* :angle-vector) av-seq)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

    (setq av-seq (reverse av-seq))
    (send *ri* :angle-vector-sequence av-seq (list 3000 2000 1000))
    (send *ri* :wait-interpolation)


    ;; release cloth
    (let (avs)
      (send *ri* :start-grasp :rarm :objects nil)
      (dotimes (i 5)
        (send *pr2* :rarm :wrist-r :joint-angle (* i 178))
        (push (send *pr2* :angle-vector) avs))
      (send-message *ri* (send *ri* :super) :angle-vector-sequence avs (make-sequence 'list (length avs) :initial-element 500))
      (send *ri* :wait-interpolation)
      (send *ri* :stop-grasp :rarm :wait nil)
      (send-message *ri* (send *ri* :super) :angle-vector-sequence (reverse avs) (make-sequence 'list (length avs) :initial-element 500))
      (send *ri* :wait-interpolation)
      )

#|
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *pr2* :rarm :wrist-r :joint-angle 0)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation)
|#
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
  t
)

(defun close-laundry ()
  (warn "close-laundry~%")
  (send *pr2* :larm :collar-y :joint-angle -90)
  (send *ri* :angle-vector (send *pr2* :angle-vector))

  (let ((handle (send *laundry* :handle-handle-cap))
	 release)

    ;; ready to close
;    (send *pr2* :larm :move-end-pos (float-vector 50 -120 0) (send *pr2* :worldcoords))

	(send *pr2* :larm :inverse-kinematics
		  (send (make-coords :pos (float-vector 900 -100 650))
				:transform *laundry* :world)
		  :rotation-axis nil
		  :debug-view nil
		  :revert-if-fail t)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (send *ri* :wait-interpolation)
    (print "ready to close")

    ;; close the cap
    (setq release (send (make-coords :pos (float-vector 650 -110 530)
				     :rpy (list (deg2rad 180) (deg2rad 0) (deg2rad 0)))
			:transform *laundry* :world))
    (warn "release: ~A~%" release)

    (send *pr2* :larm :inverse-kinematics
	  release
	  :rotation-axis t
	  :debug-view nil
	  :revert-if-fail nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation)
    )
    (pr2-reset-pose)
    t
  )

(defun push-button-laundry ()
  (warn "There is Nothing to do at func:push-button-laundry")
  t
  )