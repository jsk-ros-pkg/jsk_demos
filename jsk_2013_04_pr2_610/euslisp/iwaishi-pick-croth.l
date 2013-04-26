#!/usr/bin/env roseus
;;;;
;;;; send robot joint angle-vector to pr2 robot
;;;; pick croth by Iwaishi
;;;;

(defun iwaishi-demo()

  ;;(load "models/room610-scene.l")
  (load "package://pr2eus/pr2-interface.l")
  (load "models/room610-chair-object.l")
  (load "models/red-chair0-object.l") ;; need to use karimoku chair
  (ros::roseus "pr2_send_joints")

  (pr2)
  (if (not (boundp '*irtviewer*)) (make-irtviewer))
  ;;(setq *chair* (red-chair0))
  (setq *chair* (room610-chair))
  (send *chair* :move-to (make-coords :pos #f(1000 100 0) :rpy (float-vector 0 0 0)) :world)
  ;;(1000 -100 60)
  (objects (list *pr2* *chair*))
  (send *irtviewer* :draw-objects)

  ;;(if (not (boundp '*room610*)) (room610))
  (setq *ri* (instance pr2-interface :init))
  ;;(if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init :objects (send *room610* :objects))))

  (iwaishi-pick-cloth)
)

(defun iwaishi-pick-cloth ()
  (send *pr2* :reset-pose)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :stop-grasp :rarm)
  (send *ri* :wait-interpolation)

  ;;kamae
  (send *pr2* :angle-vector
	#f(50.0 60.0 74.0 70.0 -120.0 20.0 -30.0 180.0 -40.0 -20.0 0.0 -70.0 -70.0 -90.0 0.0 0.0 0.0))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;;grasp
  (setq *croth-point*
	(apply 'midpoint 0.5 (list
	    (send (send *chair* :handle-above-cloth-larm) :worldpos)
	    (send (send *chair* :handle-above-cloth-rarm) :worldpos))))
  (setq *croth-cascoord* (make-cascoords :pos *croth-point*))
  (send *croth-cascoord* :rotate (deg2rad 90) :y)
  (send *pr2* :inverse-kinematics *croth-cascoord*
	:link-list 
	(send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	;;(list (send *pr2* :r_shoulder_pan_link) (send *pr2* :r_shoulder_lift_link) (send *pr2* :r_upper_arm_roll_link) (send *pr2* :r_elbow_flex_link) (send *pr2* :r_forearm_roll_link) (send *pr2* :r_wrist_flex_link) (send *pr2* :r_wrist_roll_link))
	:move-target (send *pr2* :rarm :end-coords)
	:look-at-target t)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;;lift up
  (send *pr2* :angle-vector
	#f(50.0 60.0 74.0 70.0 -120.0 20.0 -30.0 180.0 -20.0 -20.0 0.0 -70.0 -70.0 -90.0 0.0 0.0 0.0))
  (send *pr2* :head :look-at (send *pr2* :rarm :end-coords :worldpos))
  (send *irtviewer* :draw-objects)
  (send *ri* :start-grasp :rarm)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
)