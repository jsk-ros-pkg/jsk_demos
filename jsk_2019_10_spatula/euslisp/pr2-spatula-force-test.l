#!/usr/bin/env roseus
;;;;
;;;; make PR2 use a spatula to scrape through a bowl without feedback, this is only a simple experiment in order to gather data for analysis of PR2 effort accuracy
;;;;

(load "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2_send_joints")
;;set this flag to decicde whether to use simulation or real robot
(setq simulation t)

 
(if simulation
    (progn
	   (pr2)
	   (when (not (boundp '*irtviewer*))
	     (make-irtviewer)
	     (objects (list *pr2*))
	     (send *irtviewer* :draw-objects))
	   
	   (setq *ri* (instance pr2-interface :init))	   
     )  
    (pr2-init)  
)

;;start robot from reset position
(send *pr2* :reset-pose)
(if simulation
    (if (boundp '*irtviewer*)
    (send *irtviewer* :draw-objects))
    )
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)


;; move robot into initial position
(send *pr2* :r_shoulder_lift_joint :joint-angle -20)
(send *pr2* :l_shoulder_lift_joint :joint-angle 50)
(send *pr2* :r_upper_arm_roll_joint :joint-angle -130)
(send *pr2* :l_upper_arm_roll_joint :joint-angle 120)
(send *pr2* :l_elbow_flex_joint :joint-angle -100)
(send *pr2* :r_elbow_flex_joint :joint-angle -70)
(send *pr2* :r_forearm_roll_joint :joint-angle 0)
(send *pr2* :r_wrist_flex_joint :joint-angle -40)
(send *pr2* :l_wrist_roll_joint :joint-angle 30)


(if simulation
    (if (boundp '*irtviewer*)
    (send *irtviewer* :draw-objects))
    )

(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

;; hand the robot the spatula and bowl
(send *pr2* :stop-grasp)
(send *ri* :stop-grasp)
(send *ri* :wait-interpolation)
(unix:sleep 5)		     
;;some time to place the bowl and then close the left gripper
(send *pr2* :start-grasp :larm)
(send *ri* :start-grasp :larm :gain 0.05)
(send *ri* :wait-interpolation)
(unix:sleep 5)
;;some time to place the spatula, then close the right gripper
(send *pr2* :start-grasp :rarm)
(print "HERE1")
(send *ri* :start-grasp :rarm :gain 0.05)
(print "HERE2")
(send *ri* :wait-interpolation)
(print "HERE3")

;;got these angle vectors from the real robot by moving it manually to the desired positions, they turned out to work better than the previously constructed
;;with: send *ri* :state :potentio-vector
(setq *av1* #f(49.9331 62.6833 33.1418 127.946 -117.444 -7.41574 -5.72958 51.833 -16.9966 -9.03369 -111.73 -116.714 -76.193 -57.7095 266.18 3.0727 -21.2682))

(setq *av2* #f(49.9331 62.3888 32.6716 129.774 -117.502 -7.41574 -5.72958 51.833 -29.1714 -9.19364 -117.398 -118.456 -81.5623 -50.4353 263.348 3.0727 -21.2682))

(setq *av3* #f(49.9331 62.6975 32.5311 129.278 -117.386 -7.41574 -5.72958 51.833 -29.0716 -9.25665 -86.0776 -118.804 -76.2328 -66.8658 270.291 3.0727 -21.2682))


(send *pr2* :angle-vector *av3*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)
;;scrape the bottom of the bowl 40 times
(dotimes (i 40)
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



