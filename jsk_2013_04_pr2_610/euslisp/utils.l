#!/usr/bin/env roseus

(defun gripper-z-axis-inversed-p (arm)
  (let ((r (send (send *pr2* arm :end-coords) :rotate-vector #f(0 0 1))))
    (if (> (elt r 2) 0) t nil)))

#|
(defun raise-mop nil
  (let ((r-grasp-height 600)
	(l-grasp-height 750)
	(raise-height 600))
;;    (send *pr2* :torso :waist-z :joint-angle 150)
;;    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)

    ;; release mop
    (send *ri* :move-gripper :rarm 0.03 :effort 20)
    (send *ri* :move-gripper :larm 0.1  :effort 20)
    (send *ri* :wait-interpolation)

    ;; down 200 mm
    (let ((rhand-z (elt (send *pr2* :rarm :end-coords :worldpos) 2))
	  (lhand-z (elt (send *pr2* :larm :end-coords :worldpos) 2)))
      (send *pr2* :rarm :move-end-pos (float-vector 0 0 (- r-grasp-height rhand-z)) :world)
      (send *pr2* :larm :move-end-pos (float-vector 0 0 (- l-grasp-height lhand-z)) :world)
      (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
      (send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 2000 :arms :rotation-axis :z)
      (send *ri* :wait-interpolation)

      )

    (send *ri* :start-grasp)

    (send *pr2* :arms :move-end-pos (float-vector 0 0 raise-height) :world)
    (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
    (send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 2000 :arms :rotation-axis :z)
    (send *ri* :wait-interpolation)
    )
  )
|#

(defun raise-mop nil
  (let ((r-grasp-height 500)
	(l-grasp-height 650)
	(raise-height 600))
;;    (send *pr2* :torso :waist-z :joint-angle 150)
;;    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)

    ;; release mop
    (send *ri* :move-gripper :rarm 0.02 :effort 20)
    (send *ri* :move-gripper :larm 0.02  :effort 20)
    (send *ri* :wait-interpolation)
    
    (send *pr2* :larm :inverse-kinematics
                (make-cascoords :pos (send *pr2* :transform-vector #f(650 0 650))
                                :rpy (list (deg2rad -90) 0 0))
                :look-at-target t
                :rotation-axis :z
                :debug-view nil)
    (send *pr2* :rarm :inverse-kinematics
                (make-coords :pos (send *pr2* :transform-vector #f(650 50 500))
                             :rpy (list (deg2rad 90) 0 0))
                :rotation-axis :z
                :debug-view nil)
    (send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 2000 :arms :rotation-axis :z)
    (send *ri* :wait-interpolation)
    (send *pr2* :angle-vector (send *pr2* :angle-vector))
    ;; down 200 mm
    (let ((rhand-z (elt (send *pr2* :rarm :end-coords :worldpos) 2))
	  (lhand-z (elt (send *pr2* :larm :end-coords :worldpos) 2)))
      (send *pr2* :rarm :move-end-pos (float-vector 0 0 (- r-grasp-height rhand-z)) :world)
      (send *pr2* :larm :move-end-pos (float-vector 0 0 (- l-grasp-height lhand-z)) :world)
      (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
      (send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 2000 :arms :rotation-axis :z)
      (send *ri* :wait-interpolation)

      )
    
    ;;grasp well
    (send *ri* :start-grasp)
    (send *ri* :start-grasp)
    
    (send *pr2* :arms :move-end-pos (float-vector 0 0 raise-height) :world :rotation-axis :z)
    (send *pr2* :head :look-at (send *pr2* :larm :end-coords :worldpos))
    (send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 2000 :arms :rotation-axis :z)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp :larm)
    (send *ri* :start-grasp :rarm :objects nil)
    )
  )

