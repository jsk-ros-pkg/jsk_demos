#!/usr/bin/env roseus

(load "package://jsk_2013_04_pr2_610/euslisp/objectdetection.l")

(defun move-to-table ()
  (warn "move-to-table~%")
  (warn "hoge~%")
  (use-tilt-laser-obstacle-cloud nil)
  (send *pr2* :reset-pose)

  ; Open collar-y a little bit
  (send *pr2* :larm :collar-y :joint-angle 75)
  (send *pr2* :rarm :collar-y :joint-angle -75)

  ; Rise shoulder-p upto horizontal position
  (send *pr2* :arms :shoulder-p :joint-angle -20)

  ; Rotate shoulder-p
  (send *pr2* :larm :shoulder-r :joint-angle 70)
  (send *pr2* :rarm :shoulder-r :joint-angle -70)

  ;; send angle-vector
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *pr2* :move-to (send *room610* :spot-tray-spot) :world)
  (prog1
      (send *ri* :move-to *tray-spot* :frame-id *room610-origin*)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (clear-costmap)
    )
  (send *pr2* :head :neck-p :joint-angle 49.386)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
)

(load "models/arrow-object.l")

(defun pick-tray ()
  (let (tray-larm-handles tray-rarm-handles avs) ;; use let to define local variables
    (warn "pick-tray (tanaka)~%")
    ;; Open gripper
    (send *ri* :stop-grasp :arms) ;; stop-grasp wait until gripper motion stops
    ;; wait interpolation
    (send *ri* :wait-interpolation)

    (setq a (arrow))
    (send a :move-to (send (send *tray* :handle-larm-handle) :worldcoords) :world)
    (objects (list *pr2* *tray* a))
    (break)
    (tray-detection *tray*)
    (send a :move-to (send (send *tray* :handle-larm-handle) :worldcoords) :world)
    (send *irtviewer* :draw-objects)
    (break)

    ;; use dual arm IK (see https://sourceforge.net/p/jsk-ros-pkg/code/4103/)
    (setq tray-larm-handle (send (send (send *tray* :handle-larm-handle) :copy-worldcoords) :rotate (deg2rad -20) :z) ;; do not move in world coords, use object(tray) relative coords
          tray-rarm-handle (send (send (send *tray* :handle-rarm-handle) :copy-worldcoords) :rotate (deg2rad  20) :z)) ;; need copy-worldcoords, otherwise move handle coords directory
    (if (boundp '*irtviewer*) (send-all (list tray-larm-handle tray-rarm-handle) :draw-on :flush t)) ;; use :draw-on to confirm coords position

    (dolist (offset (list (cons #f(-100 -10 0) #f(-100 10 0)) (cons #f(0 30 0) #f(0 -30 0))))
      (send *pr2* :inverse-kinematics
            (list
             (send (send tray-larm-handle :copy-worldcoords) :translate (car offset))  ;; use local coords, this cod works if tray is placed on different orientation
             (send (send tray-rarm-handle :copy-worldcoords) :translate (cdr offset)))
            :move-target (list (send *pr2* :larm :end-coords) (send *pr2* :rarm :end-coords))
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

    ;; Grasp it
    (send *ri* :start-grasp :arms :objects (list (find *tray* (send *ri* :objects) :test #'(lambda (a b) (string= (send a :name) (send b :name))))))  ;; stop-grasp wait until gripper motion stops
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
