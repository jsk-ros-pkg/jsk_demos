#!/usr/bin/env roseus

(ros::load-ros-manifest "ee_cart_imped_tutorial")
(load "package://roseus/euslisp/actionlib.l")
(load "package://pr2eus/pr2-interface.l")

(load "package://ee_cart_imped_tutorial/scripts/ee_cart_imped_arm.l")
;;(load "force_cul.l")
(load "models/arrow-object.l")

(warn "You need to roslaunch ee_cart_imped_tutorial change_arms_controll.launch~%")

(defun rhand-grasp nil
  (let (lhand lhand-pre)
    (send *pr2* :reset-pose)
    ;;(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *pr2* :inverse-kinematics (make-coords :pos #f(550 0 1000) :rpy (float-vector -pi/2 0 0))
        :link-list (list
                      (send *pr2* :link-list (send *pr2* :larm :end-coords :parent)))
        :move-target (list (send *pr2* :larm :end-coords))
	:rotation-axis t
        :look-at-target t
	:debug-view t
        )
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
;;    (return-from rhand-grasp)


    (send *ri* :stop-grasp :rarm)

    (setq lhand (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate #f(0 0 200) :world))
    (send lhand :orient pi/2 :z :world)
    (setq lhand-pre (send lhand :copy-worldcoords))
    (send lhand-pre :locate #f(-100 0 0))


    (send *pr2* :inverse-kinematics lhand-pre
        :link-list (list
                      (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)
			    (send *pr2* :r_shoulder_pan_link)))
        :move-target (list (send *pr2* :rarm :end-coords))
	:rotation-axis t
        :look-at-target t
	:debug-view t
        )
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (print lhand-pre)

    (send *pr2* :inverse-kinematics lhand
        :link-list (list
                      (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)
			    (send *pr2* :r_shoulder_pan_link)))
        :move-target (list (send *pr2* :rarm :end-coords))
	:rotation-axis t
        :look-at-target t
	:debug-view t
        )
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (print lhand)

    (send *ri* :start-grasp :rarm)
))

(defun lower-mop (&optional (m_arms :rarm))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;;rotational stiffness default:30
  ;;(norm force-vec-r) 1000 1000 50 50 50
  (let ((goal_l (createImpedGoal "base_footprint"))
	(goal_r (createImpedGoal "base_footprint"))
	)

;;    (setq *moving-arm* arm)
    (setq larm-end-coords (send *pr2* :larm :end-coords))
    (addForceTrajectoryCoords goal_l (send *pr2* :larm :end-coords) 1000.0
			      :control-rot (send (make-coords :axis :y :angle pi/2) :worldrot)
			      :target-force #f(-5 1000 1000 20 20 20)
			      :force-control (list t nil nil nil nil nil))

    (setq rarm-end-coords (send *pr2* :rarm :end-coords))
    (addForceTrajectoryCoords goal_r (send *pr2* :rarm :end-coords) 1000.0
			      :control-rot (send (make-coords :axis :y :angle pi/2) :worldrot)
			      :target-force #f(-5 1000 1000 20 20 20)
			      :force-control (list t nil nil nil nil nil))

    (startImpedance m_arms)
    (startTrajectory goal_l :larm)
    (startTrajectory goal_r :rarm)

    (dotimes (i 10)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (send *pr2* :head :look-at (send (send *pr2* :rarm :end-coords) :worldpos))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      (unix:sleep 1)
      )
;;    (unix:sleep 10)



    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

    ;; ;;look at object center
    ;; ;;(send *pr2* :head :look-at (send *arrow* :worldpos))
    ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    ;; (send *ri* :wait-interpolation)
    ;; (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

    )
  )

(defun pr2-rotate (rad)
  (send *pr2* :orient rad :z :world)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

  (if *world-to-610*
      (progn
	(setq current-ri-coords (send (send *world-to-610* :worldcoords) :transformation (send *ri* :state :worldcoords)))
	(setq mou-chotto-pos (v- (send (send *room610* :spot-laundry-spot) :pos) (send current-ri-coords :pos)))
	(setq mou-chotto-roll
	      (- rad (elt (elt (send current-ri-coords :rpy-angle) 0) 0)))
	(print mou-chotto-roll)
	(send *ri* :go-pos-unsafe 0 0 (rad2deg mou-chotto-roll))
	)

    (progn
      (send *ri* :move-to (send *pr2* :worldcoords))
      )
    )
  )



;; (defun pr2-move-to-unsafe (coords)
;;   (setq current-ri-coords (send (send *world-to-610* :worldcoords) :transformation (send *ri* :state :worldcoords)))
;;   (setq mou-chotto-pos (v- #f(4012.3 -1607.95 1.20346)  (send current-ri-coords :pos)))
;;   (setq mou-chotto-pos (transform (inverse-matrix  (send (send *ri* :state :worldcoords) :worldrot)) mou-chotto-pos))
;;   (setq mou-chotto-roll
;;         (- -pi/2 (elt (elt (send current-ri-coords :rpy-angle) 0) 0)))
;;   (print mou-chotto-roll)
;;   (send *ri* :go-pos-unsafe (/ (elt mou-chotto-pos 0) 1000) (/ (elt mou-chotto-pos 1) 1000)   (rad2deg mou-chotto-roll))
;; )

(defun pr2-move-to-unsafe (coords)
  (send *pr2* :move-to coords :world)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

  (if *world-to-610*
      (progn
	(setq current-ri-coords (send (send *world-to-610* :worldcoords) :transformation (send *ri* :state :worldcoords)))
	(setq mou-chotto-pos (v- (send coords :worldpos)  (send current-ri-coords :pos)))
	(setq mou-chotto-pos (transform (inverse-matrix  (send (send *ri* :state :worldcoords) :worldrot)) mou-chotto-pos))
	(setq mou-chotto-roll
	      (- -pi/2 (elt (elt (send current-ri-coords :rpy-angle) 0) 0)))
	(print mou-chotto-roll)
	(send *ri* :go-pos-unsafe (/ (elt mou-chotto-pos 0) 1000) (/ (elt mou-chotto-pos 1) 1000)   (rad2deg mou-chotto-roll))
	)
    (progn
      (send *ri* :move-to coords)
      )
    )
)

(defun move-around nil
  (setq *sweep-spot1* (make-cascoords :pos #f(4010 -1600 0) :rpy (list -pi/2 0 0)))
  (send *room610* :assoc *sweep-spot1*)
  (setq *sweep-spot2* (make-cascoords :pos #f(6510 -1607 0) :rpy (list 0 0 0)))
  (send *room610* :assoc *sweep-spot2*)
  (setq *sweep-spot3* (make-cascoords :pos #f(6510 1000 0) :rpy (list pi/2 0 0)))
  (send *room610* :assoc *sweep-spot3*)
  (setq *sweep-spot4* (make-cascoords :pos #f(4010 1000 0) :rpy (list pi 0 0)))
  (send *room610* :assoc *sweep-spot4*)

  (pr2-rotate -pi/2)
  (pr2-move-to-unsafe *sweep-spot1*)
  (pr2-rotate 0)
  (pr2-move-to-unsafe *sweep-spot2*)
  (pr2-rotate pi/2)
  (pr2-move-to-unsafe *sweep-spot3*)
  (pr2-rotate pi)
  (pr2-move-to-unsafe *sweep-spot4*)
)


(defun furuta-init nil
  (pr2)
  ;;(set-link)
  ;;(objects (list *arrow* *pr2*))
  (objects (list *pr2*))
  (if (not (boundp '*ri*))(setq *ri* (instance pr2-interface :init)))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;;debug
  ;;(marker-memory :pose-move)
)

(defun furuta-sweep nil
  (ros::roseus "read_marker_sample")
  (setq *moving-arm* :both)
  ;;(rhand-grasp)
  (lower-mop *moving-arm*)
  (move-around)
  (stopImpedance *moving-arm*)
)




(defun furuta-sweep-under-table nil
;;  (lower-mop)
;;  (setq origin (make-coords :pos #f(500 0 0)))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

  (setq origin (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
  (setq goal (send (send origin :copy-worldcoords) :locate #f(100 0 0)))
  (move-mop origin goal 1000)

  (setq goal (send (send origin :copy-worldcoords) :locate #f(-100 0 0)))
  (move-mop origin goal 1000)

  ;;(objects (list *pr2* origin))
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
;;  (print origin)
;;  (send *pr2* :rarm :end-coords)
)

(defun move-mop (nowcoords goalcoords &optional  (time 10000.0) (inter_num 10))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let ((goal_l (createImpedGoal "base_footprint"))
	(goal_r (createImpedGoal "base_footprint"))
	)
    (setq *moving-arm* :rarm)
    (setq larm-from-nowcoords (send nowcoords :transformation (send *pr2* :larm :end-coords)))
    (setq rarm-from-nowcoords (send nowcoords :transformation (send *pr2* :rarm :end-coords)))

    (dotimes (i inter_num)
      (print goalcoords)
      (print nowcoords)
      (setq middlecoords (midcoords 0.5 nowcoords goalcoords))
      (setq middlecoords (midcoords (/ (+ i 1.0) inter_num) nowcoords goalcoords))

      (print middlecoords)
      (send middlecoords :transform larm-from-nowcoords)

;;      (setq larm-end-coords (send *pr2* :larm :end-coords))
      (addForceTrajectoryCoords goal_l middlecoords (* i (/ time inter_num))
				:control-rot (send (make-coords :axis :y :angle pi/2) :worldrot)
				:target-force #f(5 1000 1000 20 20 20)
				:force-control (list t nil nil nil nil nil))

      (setq middlecoords (midcoords (/ 1.0 (+ i 1)) nowcoords goalcoords))
      (send middlecoords :transform rarm-from-nowcoords)
      (addForceTrajectoryCoords goal_r middlecoords (* i (/ time inter_num))
				:control-rot (send (make-coords :axis :y :angle pi/2) :worldrot)
				:target-force #f(5 1000 1000 20 20 20)
				:force-control (list t nil nil nil nil nil))
      )
    (startImpedance *moving-arm*)
    (startTrajectory goal_l :larm)
    (startTrajectory goal_r :rarm)
    
    (dotimes (i inter_num)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (send *pr2* :head :look-at (send (send *pr2* :rarm :end-coords) :worldpos))
      (send *ri* :angle-vector (send *pr2* :angle-vector) (/ time inter_num))
      (unix:usleep  (floor (/ (* time 1000) inter_num) ))
      )
    (unix:sleep 2)
    ;;    (unix:sleep 10)

    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
;;    (send nowcoords :locate #f(100 0 0))
    (setq renew-coord-r (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    (send renew-coord-r :transform (send rarm-from-nowcoords :inverse-transformation))
    (setq renew-coord-l (send (send *pr2* :larm :end-coords) :copy-worldcoords))
    (send renew-coord-l :transform (send larm-from-nowcoords :inverse-transformation))

    (case *moving-arm*
      (:rarm
       (send nowcoords :move-to renew-coord-r :world)
       )
      (:larm
       (send nowcoords :move-to renew-coord-l :world)
       )
      (t
       (send nowcoords :move-to (midcoords 0.5 renew-coord-r renew-coord-l) :world)
       )
      )
    )
  )
