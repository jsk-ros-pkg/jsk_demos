#!/usr/bin/env roseus
;;;;
;;;; send robot joint angle-vector to pr2 robot
;;;;

(defun move-chair ()

(load "capture_prepare_pose.l")
(send *pr2* :reset-pose)
(if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)


(send *pr2* :reset-pose)
(send *pr2* :angle-vector (collar0_more_hang_up_hawk))
(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
(send *ri* :wait-interpolation)



;;copy now hand coords
(setq destC (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
(send destC :translate #f(650 550 -400) :world)
(send destC :rotate (deg2rad 45) :y :world)
;;open right gripper

(send *ri* :stop-grasp :rarm)


;;move right hand to where grip
(send *pr2* :inverse-kinematics
	destC
      :revert-if-fail nil
      :rotation-axis t
      :move-target (send *pr2* :rarm :end-coords)
      :link-list (send *pr2* :link-list
		       (send *pr2* :rarm :end-coords :parent)
		       (send *pr2* :torso_lift_link))
      :debug-view t)
(send *pr2* :head :look-at (send *pr2* :rarm :end-coords :worldpos))
(if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
(send *ri* :wait-interpolation)

;;close right gripper

(send *ri* :start-grasp :rarm) 



;;pull chair to robot side 


(setq destC (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
(send destC :translate #f(-250 0 -200) :world)
(send destC :rotate (deg2rad -45) :y :world)

(send *pr2* :inverse-kinematics
        destC
      :revert-if-fail nil
      :rotation-axis t
      :move-target (send *pr2* :rarm :end-coords)
      :link-list (send *pr2* :link-list
                       (send *pr2* :rarm :end-coords :parent)
                       (send *pr2* :torso_lift_link))
      :debug-view t)
(send *pr2* :head :look-at (send *pr2* :rarm :end-coords :worldpos))
(if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *ri* :go-pos-unsafe -1 -1 0)
(send *ri* :wait-interpolation)



;;release chair
(setq destC (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
(send destC :translate #f(300 0 300) :world)
(send destC :rotate (deg2rad -45) :y :world)

(send *pr2* :inverse-kinematics
        destC
      :revert-if-fail nil
      :rotation-axis t
      :move-target (send *pr2* :rarm :end-coords)
      :link-list (send *pr2* :link-list
                       (send *pr2* :rarm :end-coords :parent)
                       (send *pr2* :torso_lift_link))
      :debug-view t)
(send *pr2* :head :look-at (send *pr2* :rarm :end-coords :worldpos))
(if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
(send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :rarm)


)
