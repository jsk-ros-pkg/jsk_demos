#!/usr/bin/env runeus

(load "elevator-buttons.l")
(load "package://pr2eus/pr2-utils.l")
(load "package://pr2eus/pr2-interface.l")
;(load "package://manipserver_lib/euslisp/manip_client_lib.l")

(defun push-button (target-coords)
  (let (via-coords arm
		   via-angle-vector org-arm-anglev
		   (rarm-reset #f(-60 70 110 -120 160 -30 180)) ;; second element 80 -> 70
		   (larm-reset #f(60 70 70 -120 -160 -30 180))) ;; second element 80 -> 70
    ;; via-coords
    (setq via-coords (send (send target-coords :copy-worldcoords) :translate #f(-100 0 0) :world))

    ;; check push target simply from current tuckarm-pose
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state))
    (setq arm (check-tuckarm-pose))

    (when arm
      (send *pr2* arm :angle-vector (case arm (:rarm rarm-reset)
					  (:larm larm-reset)))
      (unless (send *pr2* arm :inverse-kinematics target-coords :rotation-axis :x)
	(setq *ik-fail-av*  (send *pr2* :angle-vector))
	(setq *ik-fail-coords*  (send target-coords :copy-worldcoords))
	(dump-loadable-structure (format nil "ik-fail-~d.l" (send (ros::time-now) :to-sec))
				 *ik-fail-coords* *ik-fail-av*)
	(setq arm nil)))
    (send *pr2* :angle-vector (send *ri* :state))

    ;; reset-pose and select arm
    (unless arm
      (setq arm (if (plusp (elt (send target-coords :worldpos) 1)) :larm :rarm))
      (send *pr2* :reset-pose)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
      (send *ri* :wait-interpolation))

    (ros::ros-info "select ~A arm to push button ~A" arm (send target-coords :worldpos))

    (setq org-arm-anglev (send *pr2* arm :angle-vector))

    ;; start push
    (send *pr2* arm :angle-vector (case arm (:rarm rarm-reset)
					(:larm larm-reset)))
    (send *pr2* :head :look-at (send target-coords :worldpos))
    (unless
	(send *pr2* arm :inverse-kinematics via-coords :rotation-axis :x)
      (setq *ik-fail-av*  (send *pr2* :angle-vector))
      (setq *ik-fail-coords*  (send target-coords :copy-worldcoords))
      (dump-loadable-structure (format nil "ik-fail-~d.l" (send (ros::time-now) :to-sec))
			       *ik-fail-coords* *ik-fail-av*)
      (return-from push-button nil))
    (setq via-angle-vector (send *pr2* :angle-vector))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
    (send *ri* :wait-interpolation)

    (unless
	(send *pr2* arm :inverse-kinematics target-coords :rotation-axis :x)
      (setq *ik-fail-av*  (send *pr2* :angle-vector))
      (setq *ik-fail-coords*  (send target-coords :copy-worldcoords))
      (dump-loadable-structure (format nil "ik-fail-~d.l" (send (ros::time-now) :to-sec))
			       *ik-fail-coords* *ik-fail-av*)
      (return-from push-button nil))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)

    (send *ri* :angle-vector via-angle-vector 400)
    (send *ri* :wait-interpolation)

    ;; return original pose
    ;;(send *pr2* :head :angle-vector #f(0 0))
    (send *pr2* arm :angle-vector org-arm-anglev)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) ;; quick!
    (send *ri* :wait-interpolation)
#|
    (send *s* :arm-target-coords arm via-coords)
    (unix:sleep 10)
    (send *s* :arm-target-coords arm target-coords)
    (unix:sleep 2)
    (send *s* :arm-target-coords arm via-coords)
|#
    t))

