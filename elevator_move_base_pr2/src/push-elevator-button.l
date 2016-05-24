#!/usr/bin/env runeus

(load "elevator-buttons.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
;(load "package://manipserver_lib/euslisp/manip_client_lib.l")

(defun push-button (target-coords)
  (let (rayvec via-coords (arm :rarm)
        via-angle-vector org-arm-anglev
        (rarm-reset #f(-60 70 110 -120 160 -30 180)) ;; second element 80 -> 70
        (larm-reset #f(60 70 70 -120 -160 -30 180))) ;; second element 80 -> 70
    ;; check push target simply from current tuckarm-pose
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    ;; if the free-arm is grasping something // TODO: smart arm change motion
    (format t "grasp -> ~A~%" (send *ri* :start-grasp :arms))
    (when (< 5 (send *ri* :start-grasp :larm))
      (pr2-tuckarm-pose :rarm))
    (when (< 5 (send *ri* :start-grasp :rarm))
      (pr2-tuckarm-pose :larm))

    (setq arm (check-tuckarm-pose))
    (setq rayvec (normalize-vector (v- (send *pr2* :narrow_stereo_optical_frame_lk :worldpos) (send target-coords :worldpos))))

    ;; via-coords
    (setq via-coords (send (send target-coords :copy-worldcoords)
			   :translate (scale 100 rayvec) :world))
    ;; push 30mm
    (setq target-coords (send (send target-coords :copy-worldcoords)
			      :translate (scale -30 rayvec) :world))

    (when arm
      (send *pr2* arm :angle-vector (case arm (:rarm rarm-reset)
					  (:larm larm-reset)))
      ;; check IK for free-arm is solvable
      (unless (and arm (send *pr2* arm :inverse-kinematics target-coords :rotation-axis :x))
	(setq *ik-fail-av*  (send *pr2* :angle-vector))
	(setq *ik-fail-coords*  (send target-coords :copy-worldcoords))
	(dump-loadable-structure (format nil "ik-fail-~d.l" (send (ros::time-now) :sec))
				 *ik-fail-coords* *ik-fail-av*)
	(setq arm nil)))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    ;;

    ;; reset-pose and select arm
    (unless arm
;      (setq arm (if (plusp (elt (send target-coords :worldpos) 1)) :larm :rarm))
      (setq arm (if (plusp (elt (send (send *pr2* :transformation target-coords) :worldpos) 1)) :larm :rarm))
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
	(send *pr2* arm :inverse-kinematics via-coords :rotation-axis nil)
      (setq *ik-fail-av*  (send *pr2* :angle-vector))
      (setq *ik-fail-coords*  (send target-coords :copy-worldcoords))
      (dump-loadable-structure (format nil "ik-fail-~d.l" (send (ros::time-now) :sec))
			       *ik-fail-coords* *ik-fail-av*)
      ;; return to original pose
      (send *pr2* arm :angle-vector org-arm-anglev)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) ;; quick!
      (send *ri* :wait-interpolation)
      (return-from push-button nil))
    (setq via-angle-vector (send *pr2* :angle-vector))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
    (send *ri* :wait-interpolation)

    (send *pr2* arm :inverse-kinematics via-coords :rotation-axis :x
	  :revert-if-fail nil)
    (setq via-angle-vector (send *pr2* :angle-vector))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation)

    (unless
	(send *pr2* arm :inverse-kinematics target-coords :rotation-axis :x)
      (setq *ik-fail-av*  (send *pr2* :angle-vector))
      (setq *ik-fail-coords*  (send target-coords :copy-worldcoords))
      (dump-loadable-structure (format nil "ik-fail-~d.l" (send (ros::time-now) :sec))
			       *ik-fail-coords* *ik-fail-av*)
      ;; return to original pose
      (send *pr2* arm :angle-vector org-arm-anglev)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) ;; quick!
      (send *ri* :wait-interpolation)
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

