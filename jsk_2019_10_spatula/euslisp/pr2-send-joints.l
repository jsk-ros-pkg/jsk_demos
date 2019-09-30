#!/usr/bin/env roseus
;;;;
;;;; send robot joint angle-vector to pr2 robot
;;;;

(load "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2_send_joints")

(pr2)
(when (not (boundp '*irtviewer*))
  (make-irtviewer)
  (objects (list *pr2*))
  (send *irtviewer* :draw-objects))

(setq *ri* (instance pr2-interface :init))

(send *pr2* :reset-pose)
(if (boundp '*irtviewer*)
    (send *irtviewer* :draw-objects))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *pr2* :torso :waist-z :joint-angle 100)
(send *pr2* :larm :collar-y :joint-angle  90)
(send *pr2* :rarm :collar-y :joint-angle -90)
(send *pr2* :arms :shoulder-p :joint-angle 0)
(send *pr2* :arms :shoulder-r :joint-angle 0)
(send *pr2* :arms :elbow-p :joint-angle -90)
(send *pr2* :arms :elbow-r :joint-angle 0)
(send *pr2* :head :neck-p :joint-angle -10)
(if (boundp '*irtviewer*)
    (send *irtviewer* :draw-objects))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *pr2* :reset-pose)
(send *pr2* :rarm :move-end-pos #f(0  170 400) :world :rotation-axis nil)
(send *pr2* :larm :move-end-pos #f(0 -170 400) :world :rotation-axis nil)
(send *pr2* :head :look-at (apply 'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos)))
(if (boundp '*irtviewer*)
    (send *irtviewer* :draw-objects))
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *pr2* :inverse-kinematics
      (make-coords :pos #f(400 300 1600))
      :rotation-axis nil
      :move-target (send *pr2* :larm :end-coords)
      :link-list (send *pr2* :link-list
		       (send *pr2* :larm :end-coords :parent)
		       (send *pr2* :torso_lift_link_lk))
      :debug-view t)
(send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
(if (boundp '*irtviewer*)
    (send *irtviewer* :draw-objects))
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)
