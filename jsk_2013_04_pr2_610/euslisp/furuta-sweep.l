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



(defun force-move (&key (mem t))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;;rotational stiffness default:30
  ;;(norm force-vec-r) 1000 1000 50 50 50
  (let ((goal_l (createImpedGoal "base_footprint"))
	(goal_r (createImpedGoal "base_footprint"))
	)

    (setq *moving-arm* :both)
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

    (startImpedance *moving-arm*)
    (startTrajectory goal_l :larm)
    (startTrajectory goal_r :rarm)

    (dotimes (i 10)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
      (send *pr2* :head :look-at (send (send *pr2* :rarm :end-coords) :worldpos))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      (unix:sleep 1)
      )
;;    (unix:sleep 10)

    (stopImpedance *moving-arm*)

    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

    ;; ;;look at object center
    ;; ;;(send *pr2* :head :look-at (send *arrow* :worldpos))
    ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    ;; (send *ri* :wait-interpolation)
    ;; (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

    )
  )

(defun stop-force nil
    (setq req (instance ee_cart_imped_tutorial::UseImpedRequest :init :useImped nil))
    (ros::service-call "r_imped_control" req)
    (ros::service-call "l_imped_control" req)

    (print "exe")
    (unix::sleep 1)
 ;;   (send *ri* :wait-interpolation)
    ;;*pr2*への反映
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
)

(defun pr2-rotate (rad)
  (setq current-ri-coords (send (send *world-to-610* :worldcoords) :transformation (send *ri* :state :worldcoords)))
  (setq mou-chotto-pos (v- (send (send *room610* :spot-laundry-spot) :pos) (send current-ri-coords :pos)))
  (setq mou-chotto-roll
        (- rad (elt (elt (send current-ri-coords :rpy-angle) 0) 0)))
  (print mou-chotto-roll)
  (send *ri* :go-pos-unsafe 0 0 (rad2deg mou-chotto-roll))
)

(defun pr2-move-first nil
  (setq current-ri-coords (send (send *world-to-610* :worldcoords) :transformation (send *ri* :state :worldcoords)))
  (setq mou-chotto-pos (v- #f(4012.3 -1607.95 1.20346)  (send current-ri-coords :pos)))
  (setq mou-chotto-pos (transform (inverse-matrix  (send (send *ri* :state :worldcoords) :worldrot)) mou-chotto-pos))
  (setq mou-chotto-roll
        (- -pi/2 (elt (elt (send current-ri-coords :rpy-angle) 0) 0)))
  (print mou-chotto-roll)
  (send *ri* :go-pos-unsafe (/ (elt mou-chotto-pos 0) 1000) (/ (elt mou-chotto-pos 1) 1000)   (rad2deg mou-chotto-roll))
)

(defun move-around nil

(pr2-rotate -pi/2)
(pr2-move-first)

  (pr2-rotate 0)
;;  (return-from move-around)
	(send *ri* :go-pos-unsafe 0 0 90)
	(send *ri* :go-pos-unsafe 2.5 0 0)
	(send *ri* :go-pos-unsafe 0 0 90)

	(send *ri* :go-pos-unsafe 2.5 0 0)
	(send *ri* :go-pos-unsafe 0 0 90)

	(send *ri* :go-pos-unsafe 2.5 0 0)
	(send *ri* :go-pos-unsafe 0 0 90)

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
  ;;(rhand-grasp)
  (force-move)
  (move-around)
  (stop-force)
)