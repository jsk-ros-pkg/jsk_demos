#!/usr/bin/env roseus

(ros::load-ros-manifest "elevator_move_base_pr2")

(ros::roseus "find_elevator_button")

(load "package://elevator_move_base_pr2/src/posedetectiondb-client.l")
(load "package://elevator_move_base_pr2/src/elevator-buttons.l")

(load "package://pr2eus/pr2-utils.l")
(load "package://pr2eus/pr2-interface.l")


(setq *scene* (make-eng8-scene))
(setq *clients* (init-posedetectiondb-client (send *scene* :objects)))
;;(setq *ri* (instance pr2-interface :init))

(setq *node-debug* nil)
(when *node-debug*
  (setq *ri* (instance pr2-interface :init))
  (setq *pr2* (pr2))
  (defvar *tfl* (instance ros::transform-listener :init))
  (if (not (boundp '*irtviewer*)) (make-irtviewer :title "find-elevator-button"))
  (objects (append (list *pr2*) (send *scene* :objects)))
  (send *irtviewer* :look-all *pr2*)
  )

(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (when *node-debug*
   (send *pr2* :angle-vector (send *ri* :state))
   (send *irtviewer* :draw-objects)
   (let ((camera-coords (send *tfl* :lookup-transform "/base_footprint" "/narrow_stereo_optical_frame" (ros::time 0))))
     (when camera-coords
       (send camera-coords :draw-on :flush t :size 200)))
   (x::window-main-one)
   )
 (ros::sleep)
 )

