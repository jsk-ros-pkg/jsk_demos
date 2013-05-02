#!/usr/bin/env roseus

(defun move-to-table ()
  (warn "move-to-table~%")
  (warn "hoge~%")
;  (speak-jp "トレイの前に移動します。")
  (use-tilt-laser-obstacle-cloud nil)
  (send *pr2* :reset-pose)
                                        ;  (pr2-tuckarm-pose)
;  (send *pr2* :torso :waist-z :joint-angle 150) ;; move torso while walking to table spot?

                                        ; Open collar-y a little bit
  (send *pr2* :larm :collar-y :joint-angle 75)
  (send *pr2* :rarm :collar-y :joint-angle -75)
                                        ; Rise shoulder-p upto horizontal position
  (send *pr2* :arms :shoulder-p :joint-angle -20)
                                        ; Rotate shoulder-p
  (send *pr2* :larm :shoulder-r :joint-angle 70)
  (send *pr2* :rarm :shoulder-r :joint-angle -70)

  ;; send angle-vector
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
  (send *pr2* :move-to (send *room610* :spot-tray-spot) :world)
  (send *ri* :move-to *tray-spot* :frame-id *room610-origin*)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
  (use-tilt-laser-obstacle-cloud t)
;  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
;  (send *ri* :wait-interpolation)
)

(defun pick-tray ()
  (let (tray-larm-handles tray-rarm-handles avs) ;; use let to define local variables
    (warn "pick-tray (tanaka)~%")
    (send *pr2* :move-to (send *room610* :spot-tray-spot) :world)

;    (speak-jp "トレイを持ちます。")

                                        ; setup initial posture
                                        ; Use torso
    (send *pr2* :reset-pose)
    ;; Open gripper
    (send *ri* :stop-grasp :arms) ;; stop-grasp wait until gripper motion stops
    ;; wait interpolation
    (send *ri* :wait-interpolation)

    ;; use dual arm IK (see https://sourceforge.net/p/jsk-ros-pkg/code/4103/)
    (setq tray-larm-handle (send (send *tray* :handle-larm-handle :copy-worldcoords) :rotate (deg2rad -20) :z) ;; do not move in world coords, use object(tray) relative coords
          tray-rarm-handle (send (send *tray* :handle-rarm-handle :copy-worldcoords) :rotate (deg2rad  20) :z)) ;; need copy-worldcoords, otherwise move handle coords directory
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

      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )

    ;; Grasp it
    (send *ri* :start-grasp :arms)  ;; stop-grasp wait until gripper motion stops
    (send *pr2* :larm :end-coords :assoc *tray*) ;; ???

    (setq avs nil)
    ;; Hold it up!
    (send *pr2* :arms :move-end-pos #f(0 0 100) :world)
    (send *pr2* :look-at-hand :arms)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (push (send *pr2* :angle-vector) avs)

    ;; Move it close to PR2
    (send *pr2* :arms :move-end-pos #f(0 0 -250))
    (send *pr2* :look-at-hand :arms)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    (push (send *pr2* :angle-vector) avs)

    (setq avs (reverse avs))

    (send *ri* :angle-vector-sequence avs 1000) ;; use angle-vector-sequence
    (send *ri* :wait-interpolation)

    ;; Go back 50cm
    (speak-en "I'm going back ATTENTION!")
    (send *ri* :go-pos-unsafe -0.5 0 30)
    t))
