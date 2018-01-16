#!/usr/bin/env roseus

(ros::load-ros-manifest "elevator_move_base_pr2")

(ros::roseus "find_elevator_button")

(load "package://elevator_move_base_pr2/src/posedetectiondb-client.l")
(load "package://elevator_move_base_pr2/src/elevator-buttons.l")

(load "package://pr2eus/pr2-interface.l")

(defvar *tfl* (instance ros::transform-listener :init))
(unix::sleep 1) ;; wait for TF messages

(let ((frames (send *tfl* :get-frame-strings)))
  (cond ((find "eng2" frames :test #'string=)
	 (setq *scene* (make-eng2-scene)))
	((find "eng8" frames :test #'string=)
	 (setq *scene* (make-eng8-scene)))
	(t
	 (ros::ros-error "There is no building frame-id in TF tree exiting...")
	 (exit))
	))
(let ((objs (list (send *scene* :object "/elevator_inside_panel")
		  (send *scene* :object "/elevator_call_panel"))))
  (setq *clients* (init-posedetectiondb-client objs)))
;;(setq *ri* (instance pr2-interface :init))

(defun display-available-p ()
  (and x::*display* (> x::*display* 0)))

(setq *node-debug* nil)
(when *node-debug*
  (setq *ri* (instance pr2-interface :init))
  (setq *pr2* (pr2))
  (when (display-available-p)
    (if (not (boundp '*irtviewer*)) (make-irtviewer :title "find-elevator-button"))
    (objects (append (list *pr2*) (send *scene* :objects)))
    (send *irtviewer* :look-all *pr2*))
  )

(ros::rate 10)
(while (ros::ok)
 (ros::spin-once)
 (when *node-debug*
   (send *pr2* :angle-vector (send *ri* :state))
   (when (display-available-p)
     (send *irtviewer* :draw-objects)
     (let ((camera-coords (send *tfl* :lookup-transform "base_footprint" "narrow_stereo_optical_frame" (ros::time 0))))
       (when camera-coords
         (send camera-coords :draw-on :flush t :size 200)))
     (x::window-main-one))
   )
 (ros::sleep)
 )

