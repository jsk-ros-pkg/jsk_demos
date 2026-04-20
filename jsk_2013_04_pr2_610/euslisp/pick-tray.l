#!/usr/bin/env roseus

(load "package://jsk_2013_04_pr2_610/euslisp/objectdetection.l")
(load "package://pr2eus/pr2-utils.l")

;; (send *pr2* :move-to (send *room610* :spot-tray-spot) :world)
(defun pick-tray-table-side ()
  (let (tray-larm-handle tray-rarm-handle 
        (grasp-threshold (list 3 3))
        (avs nil)
        (grasp-args)) ;; use let to define local variables
    (warn "pick-tray-table-side ~%")

    (pr2-pick-tray-pose)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    
    ;;detect tray when real robot
    (if (not (send *ri* :simulation-modep))
	(tray-detection *tray*)
      )
    (setq tray-larm-handle (send (send (send *tray* :handle-larm-handle) :copy-worldcoords) :rotate (deg2rad -20) :z) ;; do not move in world coords, use object(tray) relative coords
	  tray-rarm-handle (send (send (send *tray* :handle-rarm-handle) :copy-worldcoords) :rotate (deg2rad  20) :z)) ;; need copy-worldcoords, otherwise move handle coords directory


    ;; Open gripper
    (send *ri* :stop-grasp :arms) ;; stop-grasp wait until gripper motion stops
    ;; wait interpolation

    ;; use dual arm IK (see https://sourceforge.net/p/jsk-ros-pkg/code/4103/)
    (if (boundp '*irtviewer*) (send-all (list tray-larm-handle tray-rarm-handle) :draw-on :flush t)) ;; use :draw-on to confirm coords position

    (dolist (offset (list (cons #f(-100 -10 0) #f(-100 10 0)) (cons #f(0 30 0) #f(0 -30 0))))
      (send *pr2* :inverse-kinematics
            (list
             (send (send tray-larm-handle :copy-worldcoords) :translate (car offset))  ;; use local coords, this cod works if tray is placed on different orientation
             (send (send tray-rarm-handle :copy-worldcoords) :translate (cdr offset)))
            :use-torso t
            :look-at-target (midpoint 0.5 (send tray-larm-handle :worldpos) (send tray-rarm-handle :worldpos))
            )
      (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

      (print (send *pr2* :angle-vector))
      (print (send *pr2* :arms :wrist-r :joint-angle))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      ;;(send *ri* :angle-vector-with-constraint (send *pr2* :angle-vector) 1000 :arms)
      (send *ri* :wait-interpolation)
      )

    (warning-message 3 "grasp tray~%")
    ;; Grasp it
    (setq grasp-args (send *ri* :start-grasp :arms :gain 0.08 :objects (list (find *tray* (send *ri* :objects) :test #'(lambda (a b) (string= (send a :name) (send b :name)))))))  ;; stop-grasp wait until gripper motion stops
	;; check if tray is grasped
    (warning-message 3 "check tray ~A > ~A~%" grasp-args grasp-threshold)
    (when (and (not (send *ri* :simulation-modep))
	       (or (< (elt grasp-args 0) (elt grasp-threshold 0))
		   (< (elt grasp-args 1) (elt grasp-threshold 1))))
      (warning-message 3 "tray is not grasped~%")
      (speak-en "can not grasp tray")
      ;; Open gripper
      (send *ri* :stop-grasp :arms) ;; stop-grasp wait until gripper motion stops
      (pr2-pick-tray-pose)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (return-from pick-tray-table-side nil)
      )
    (send *pr2* :larm :end-coords :assoc *tray*) ;; ???

    (setq avs nil)
    ;; Hold it up!
    (send *pr2* :arms :move-end-pos #f(0 0 100) :world)
    (send *pr2* :look-at-hand :arms)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (print (send *pr2* :angle-vector))
    (print (send *pr2* :arms :wrist-r :joint-angle))
    (push (send *pr2* :angle-vector) avs)

    ;; Move it close to PR2
    (send *pr2* :arms :move-end-pos #f(0 0 -250))
    (send *pr2* :look-at-hand :arms)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (print (send *pr2* :angle-vector))
    (print (send *pr2* :arms :wrist-r :joint-angle))
    (push (send *pr2* :angle-vector) avs)

    (setq avs (reverse avs))

    (send *ri* :angle-vector-sequence avs 1000) ;; use angle-vector-sequence
    (send *ri* :wait-interpolation)

    ;; Go back 50cm
    (send *ri* :go-pos-unsafe -0.4 0 90)
    t))
