(load "test-finger.l")
(defun r-grasp nil (move-gripper 0 :effort 8.0))
(defun r-open nil (move-gripper 0.2 :effort 8.0)) ;; is max ?

(defun get-obj-coords nil
  ;; return dummy obj coords
  (coords)
  )

(defun pick (arm obj place)
  (let (target-coords target-coords-before)
    (setq terget-coords (get-obj-coords)) ;;todo
    (setq terget-coords-before
	  (send (send target-coords :copy-coords) :translate #f(200 0 0)))
    (if (eq arm 'rarm)
	(send *pr2* :inverse-kinematics :rarm-target target-coords-before)
      (send *pr2* :inverse-kinematics :larm-target target-coords-before))
    (r-open)
    (if (eq arm 'rarm)
	(send *pr2* :inverse-kinematics :rarm-target target-coords)
      (send *pr2* :inverse-kinematics :larm-target target-coords))
    (r-grasp)
    (send *pr2* :reset-pose)))

(defun put (arm obj place)
  (let (target-coords target-coords-before)
    (setq terget-coords (make-coords :pos (float-vector 600 0 800)))
    (setq terget-coords-before
	  (send (send target-coords :copy-coords) :translate #f(0 0 -300)))
    (if (eq arm 'rarm)
	(send *pr2* :inverse-kinematics :rarm-target target-coords-before)
      (send *pr2* :inverse-kinematics :larm-target target-coords-before))
    (if (eq arm 'rarm)
	(send *pr2* :inverse-kinematics :rarm-target target-coords)
      (send *pr2* :inverse-kinematics :larm-target target-coords))
    (r-open)))

(defun pour (from to from-arm to-arm)
  (let (from-init from-pour to-target
		  (arm-side (if (eq from-arm `rarm) 1 -1)))
    (setq from-init (make-coords :pos (float-vector 600 (* arm-side 100) 700)
				 :rpy (float-vector 0 0 0)))
    (setq from-pour (make-coords :pos (float-vector 600 (* arm-side 50) 800)
				 :rpy (float-vector (* arm-side 1.57) (* arm-side 1.4) 0)))
    (setq to-target (make-coords :pos (float-vector 600 0 700)
				 :rpy (float-vector 0 0 0)))
    (if (eq from-arm 'rarm)
	(send *pr2* :inverse-kinematics :larm-target to-target)
      (send *pr2* :inverse-kinematics :rarm-target to-target))

    (if (eq from-arm 'rarm)
	(send *pr2* :inverse-kinematics :rarm-target from-init)
      (send *pr2* :inverse-kinematics :larm-target from-init))
    (if (eq from-arm 'rarm)
	(send *pr2* :inverse-kinematics :rarm-target from-pour)
      (send *pr2* :inverse-kinematics :larm-target from-pour))
    (if (eq from-arm 'rarm)
	(send *pr2* :inverse-kinematics :rarm-target from-init)
      (send *pr2* :inverse-kinematics :larm-target from-init))

    (send *pr2* :reset-pose)
    ))

;; (move from to)  ;; desk1 desk2
(load "pr2_nav.l")

(while (ros::ok)
  (ros::spin-once))
