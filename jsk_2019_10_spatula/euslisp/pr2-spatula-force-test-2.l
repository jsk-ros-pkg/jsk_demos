#!/usr/bin/env roseus
;;;;
;;;; send robot joint angle-vector to pr2 robot
;;;;

(load "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2_send_joints")

;;use this for simulation

(pr2)
(when (not (boundp '*irtviewer*))
  (make-irtviewer)
  (objects (list *pr2*))
  (send *irtviewer* :draw-objects))
(setq *ri* (instance pr2-interface :init))

;;change to this, when using the real robot
;(pr2-init)


;;start robot from reset position
(send *pr2* :reset-pose)
(if (boundp '*irtviewer*)
    (send *irtviewer* :draw-objects))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)


;; move robot into initial position
(send *pr2* :r_shoulder_lift_joint :joint-angle -20)
(send *pr2* :l_shoulder_lift_joint :joint-angle 50)
(send *pr2* :r_upper_arm_roll_joint :joint-angle -130)
(send *pr2* :l_upper_arm_roll_joint :joint-angle 120)
(send *pr2* :l_elbow_flex_joint :joint-angle -100)
(send *pr2* :r_elbow_flex_joint :joint-angle -80)
(send *pr2* :r_forearm_roll_joint :joint-angle 0)
(send *pr2* :r_wrist_flex_joint :joint-angle -70)
(send *pr2* :l_wrist_roll_joint :joint-angle 30)

(if (boundp '*irtviewer*)
    (send *irtviewer* :draw-objects))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;; make coordinate systems in TCP
(setq *l-tcp* (send *pr2* :larm :end-coords))
(objects *l-tcp*)
(setq *r-tcp* (send *pr2* :rarm :end-coords))
(objects *r-tcp*)

;;move from left tcp to approximate edge of bowl
(send *l-tcp* :translate (float-vector 180 0 0))
(send *l-tcp* :translate (float-vector 0 200 0))
(send *l-tcp* :rotate pi :x)
(send *l-tcp* :rotate (/ (* 3 pi) 4) :z)

(send *l-tcp* :translate (float-vector 0 0 50))

;;use inverse kinematics to approach the bowl with the robots right hand
(send *pr2* :inverse-kinematics
      *l-tcp*
      :rotation-axis t
      :move-target (send *pr2* :rarm :end-coords)
      :use-torso nil)

(if (boundp '*irtviewer*)
    (send *irtviewer* :draw-objects))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;;;;;;;;;;;;;;;;;;;;;;;
;;add the gripper movements later
;;;;;;;;;;;;;;;;;;;;;;;

;; hand the robot the spatula and bowl
(send *pr2* :stop-grasp)
(send *ri* :stop-grasp)
(unix:sleep 5)					;(send *ri* :wait-interpolation)

;;some time to place the bowl and then close the left gripper

(send *pr2* :start-grasp :larm)
(send *ri* :start-grasp :larm)
(send *ri* :wait-interpolation)
(unix:sleep 5)
;;some time to place the spatula, then close the right gripper

(send *pr2* :start-grasp :rarm)
(send *ri* :start-grasp :rarm)
(send *ri* :wait-interpolation)


;;use inverse kinematics to make the scraping movement
;;move *l-tcp* back to the left tcp
(send *l-tcp* :translate (float-vector 0 0 -50))
(send *l-tcp* :rotate (- 0 (/ (* 3 pi) 4)) :z)
(send *l-tcp* :rotate (- 0 pi) :x)
(send *l-tcp* :translate (float-vector 0 -200 0))
(send *l-tcp* :translate (float-vector -180 0 0))


;;got these angle vectors from the robot 
(setq *av1* #f(49.9331 62.6833 33.1418 127.946 -117.444 -7.41574 -5.72958 51.833 -16.9966 -9.03369 -111.73 -116.714 -76.193 -57.7095 266.18 3.0727 -21.2682))

(setq *av2* #f(49.9331 62.3888 32.6716 129.774 -117.502 -7.41574 -5.72958 51.833 -29.1714 -9.19364 -117.398 -118.456 -81.5623 -50.4353 263.348 3.0727 -21.2682))

(setq *av3* #f(49.9331 62.6975 32.5311 129.278 -117.386 -7.41574 -5.72958 51.833 -29.0716 -9.25665 -86.0776 -118.804 -76.2328 -66.8658 270.291 3.0727 -21.2682))

(dotimes (i 20)
	(send *pr2* :angle-vector *av1*)
	(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
	(send *ri* :wait-interpolation)
	(send *pr2* :angle-vector *av2*)
	(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
	(send *ri* :wait-interpolation)
	(send *pr2* :angle-vector *av3*)
	(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
	(send *ri* :wait-interpolation)
	)

#|
;;old version
;;;;;;;;;;;;;;;;;
;;later a loop should be here to scrape again and again
;;;;;;;;;;;;;;;;;;
(send *l-tcp* :translate (float-vector 100 200 0))
(dotimes (i 4)
  ;;move the right arm to the start position
  (send *l-tcp* :rotate pi :x)
  (send *l-tcp* :rotate (/ (* 3 pi) 4) :z)
  (send *pr2* :inverse-kinematics
	*l-tcp*
	:rotation-axis t
	:move-target (send *pr2* :rarm :end-coords)
	:use-torso nil)
  (send *l-tcp* :rotate (- 0 (/ (* 3 pi) 4)) :z)
  (send *l-tcp* :rotate (- 0 pi) :x)
  (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;;lower the arm
  (send *l-tcp* :translate (float-vector 0 -200 0))
  (send *l-tcp* :rotate pi :x)
  (send *l-tcp* :rotate (/ (* 3 pi) 4) :z)
  (send *pr2* :inverse-kinematics
	*l-tcp*
	:rotation-axis t
	:move-target (send *pr2* :rarm :end-coords)
	:use-torso nil)
  (send *l-tcp* :rotate (- 0 (/ (* 3 pi) 4)) :z)
  (send *l-tcp* :rotate (- 0 pi) :x)
  (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;;scrape along bottom
  (send *l-tcp* :translate (float-vector 150 0 0))
  (send *l-tcp* :rotate pi :x)
  (send *l-tcp* :rotate (/ (* 3 pi) 4) :z)
  (send *pr2* :inverse-kinematics
	*l-tcp*
	:rotation-axis t
	:move-target (send *pr2* :rarm :end-coords)
	:use-torso nil)
  (send *l-tcp* :rotate (- 0 (/ (* 3 pi) 4)) :z)
  (send *l-tcp* :rotate (- 0 pi) :x)

  (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;;lift arm
  (send *l-tcp* :translate (float-vector 0 200 0))
  (send *l-tcp* :rotate pi :x)
  (send *l-tcp* :rotate (/ (* 3 pi) 4) :z)
  (send *pr2* :inverse-kinematics
	*l-tcp*
	:rotation-axis t
	:move-target (send *pr2* :rarm :end-coords)
	:use-torso nil)
  (send *l-tcp* :rotate (- 0 (/ (* 3 pi) 4)) :z)
  (send *l-tcp* :rotate (- 0 pi) :x)

  (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *l-tcp* :translate (float-vector -150 0 0))
)

(unix:sleep 4)

|#

#|
(send *pr2* :angle-vector *av1*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *pr2* :angle-vector *av2*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *pr2* :angle-vector *av3*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)
|#
