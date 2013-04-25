#!/usr/bin/env roseus

(ros::load-ros-manifest "ee_cart_imped_tutorial")
(load "package://roseus/euslisp/actionlib.l")
(load "package://pr2eus/pr2-interface.l")

(load "package://ee_cart_imped_tutorial/scripts/ee_cart_imped_arm.l")
;;(load "force_cul.l")
(load "models/arrow-object.l")

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
    (send *irtviewer* :draw-objects)
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
    (send *irtviewer* :draw-objects)
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
    (send *irtviewer* :draw-objects)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (print lhand)

    (send *ri* :start-grasp :rarm)
))



(defun force-move (&key (mem t))
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;;rotational stiffness default:30
  ;;(norm force-vec-r) 1000 1000 50 50 50
  (let ((imped_goal_r (instance ee_cart_imped_msgs::EECartImpedActionGoal :init))
	(imped_goal_l (instance ee_cart_imped_msgs::EECartImpedActionGoal :init))
	)
    (setq *moving-arm* :arms)
    
    (setq traj-coord-l (send *pr2* :larm :end-coords))
    (setq traj-coord-r (send *pr2* :rarm :end-coords))

    (setq tmp-pos (send traj-coord-l :worldpos))
    (setq tmp-qua (matrix2quaternion (send traj-coord-l :worldrot)))

;;    (setq *constrainted-force* (send *arrow-origin* :transformation *force-coord*))
;;    (send *constrainted-force* :transform add-coord :world)
    (setq *constrainted-force* (make-coords :rot #2f((0.0 0.0 1.0) (0.0 1.0 0.0) (-1.0 0.0 0.0)))) ;;down force
    (setq force-qua-l (matrix2quaternion (send *constrainted-force* :worldrot)))
    (addTrajectoryPoint imped_goal_l (/ (elt tmp-pos 0) 1000) (/ (elt tmp-pos 1) 1000) (/ (elt tmp-pos 2) 1000)
			(elt tmp-qua 1) (elt tmp-qua 2) (elt tmp-qua 3) (elt tmp-qua 0)
			5 1000 1000 30 30 30
			t nil nil nil nil nil
			(aref force-qua-l 1) (aref force-qua-l 2) (aref force-qua-l 3) (aref force-qua-l 0)
			0  "/base_link")
    ;;			  (/ (* i 10.0) inter-num)  "/base_link")

    (setq *constrainted-force* (make-coords :rot #2f((0.0 0.0 1.0) (0.0 1.0 0.0) (-1.0 0.0 0.0)))) ;;down force
    
	(setq tmp-pos (send traj-coord-r :worldpos))
    (setq tmp-qua (matrix2quaternion (send traj-coord-r :worldrot)))

  (setq force-qua-r (matrix2quaternion (send *constrainted-force* :worldrot)))
    (addTrajectoryPoint imped_goal_r (/ (elt tmp-pos 0) 1000) (/ (elt tmp-pos 1) 1000) (/ (elt tmp-pos 2) 1000)
			(elt tmp-qua 1) (elt tmp-qua 2) (elt tmp-qua 3) (elt tmp-qua 0)
			5 1000 1000 30 30 30
			t nil nil nil nil nil
			(aref force-qua-l 1) (aref force-qua-l 2) (aref force-qua-l 3) (aref force-qua-l 0)
			0  "/base_link")
 (print tmp-pos)
;;(return-from force-move)
    ;; (send *pr2* :head :look-at
    ;; 	    (send add-coord :worldpos))
    ;; (push (send *pr2* :angle-vector) vector-list)
    ;; (objects (list *pr2* add-coord))
    ;; ;;      (print add-coord)
    ;; (send *irtviewer* :draw-objects)
    ;; (setq reset-coord (send add-coord :copy-worldcoords))
    ;; )

    (addForceCoord imped_goal_r "/force_arrow")
    (addForceCoord imped_goal_l "/force_arrow")
    (setq req (instance ee_cart_imped_tutorial::UseImpedRequest :init :useImped t))

    (case *moving-arm*
      (:rarm
       (ros::service-call "r_imped_control" req)
       (unix::sleep 1)
       (startTrajectory imped_goal_r :rarm 0)
       )
      (:larm
       (ros::service-call "l_imped_control" req)
       (unix::sleep 1)
       (startTrajectory imped_goal_l :larm 0)
       )
      (t
       (ros::service-call "r_imped_control" req)
       (ros::service-call "l_imped_control" req)
       (unix::sleep 1)
       (startTrajectory imped_goal_r :rarm 0)
       (startTrajectory imped_goal_l :larm 0)
       )
      )
;;(send *ri* :go-pos-unsafe 0 0.2 0)
    (return-from force-move)

    ;; (unix::sleep 23)
    ;; (setq req (instance ee_cart_imped_tutorial::UseImpedRequest :init :useImped nil))
    ;; (ros::service-call "r_imped_control" req)
    ;; (ros::service-call "l_imped_control" req)

    ;; (print "exe")


    ;; (send *ri* :wait-interpolation)
    ;; ;;*pr2*への反映
    ;; (send *pr2* :angle-vector (send *ri* :state :potentio-vector))

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


(defun move-around nil
	(send *ri* :go-pos-unsafe 0.5 0 0)
	(send *ri* :go-pos-unsafe 0 0 0)
)

(ros::roseus "read_marker_sample")


(defun init nil
  (pr2)
  ;;(set-link)
  ;;(objects (list *arrow* *pr2*))
  (objects (list *pr2*))
  (if (not (boundp '*ri*))(setq *ri* (instance pr2-interface :init)))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;;debug
  ;;(marker-memory :pose-move)
)

(defun sweep nil
  (rhand-grasp)
  (force-move)
  (move-around)
  (stop-force)
)