#!/usr/bin/env roseus

(load "models/room610-scene.l")
(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")
(load "models/room73b2-broom-object.l")
(load "models/room610-scene.l")
(load "package://jsk_2013_04_pr2_610/euslisp/utils.l")
(load "package://jsk_2013_04_pr2_610/euslisp/pick-cloth.l")
(load "package://jsk_2013_04_pr2_610/euslisp/pick-tray.l")
(load "package://jsk_2013_04_pr2_610/euslisp/move-chair.l")
(load "package://jsk_2013_04_pr2_610/euslisp/place-tray.l")
(load "package://jsk_2013_04_pr2_610/euslisp/sweep-under-table.l")
(load "package://jsk_2013_04_pr2_610/euslisp/pick-broom.l")
(load "package://jsk_2013_04_pr2_610/euslisp/put-cloth-into-laundry.l")
;(load "package://jsk_2013_04_pr2_610/euslisp/detect-chair.l")
;(load "package://jsk_2013_04_pr2_610/euslisp/sweep-floor.l")

(defun setup ()
  (room610)
  (pr2)

  (objects (list *room610* *pr2*))

  (setq *room610-origin* "/eng8/6f/610")
  (setq *table-coords* (make-coords :pos #f(4700 1000 0) :rpy (float-vector -pi/2 0 0)))
  (setq *chair-coords* (make-coords :pos #f(3800 0 0)))

  (setq *tray* (send *room610* :object "room610-tray"))
  (send *tray* :locate #f(5050.0 0.0 700.0) :world)
  (setq *broom* (send *room610* :object "room610-sweeper"))
  (send *broom* :locate #f(3000.0 2090.0 0.0) :world)
  ;;(send *broom* :translate #f(0 100 0) :world)
  (setq *chair* (send *room610* :object "room610-chair"))
  (send *chair* :locate #f(4700.0 -360 0.0) :world)
  (setq *laundry* (send *room610* :object "room610-laundry-machine"))
  ;; yukizaki?
;  (send *laundry* :translate #f(-120 10 -85) (send *room610* :worldcoords))
  (send *laundry* :translate #f(-260 -30 -125) (send *room610* :worldcoords))

  ;; spots
  (setq *tray-spot* (make-cascoords :pos #f(5050 700 0) :rpy (list -pi/2 0 0)))
  (send *room610* :assoc *tray-spot*)
  (setq *chair-spot* (make-cascoords :pos #f(3850 50 0) :rpy (list 0 0 0)))
  (send *room610* :assoc *chair-spot*)
  (setq *chair-spot2* (make-cascoords :pos #f(3900 50 0) :rpy (list 0 0 0)))
  (send *room610* :assoc *chair-spot2*)
;;  (setq *laundry-spot* (make-cascoords :pos #f(3176.358 1409.163 -0.854) :rpy #f(1.544 0.004 0.002)))
;;  (setq *laundry-spot* (make-cascoords :pos #f(3376.358 1309.163 -0.854) :rpy #f(1.544 0.004 0.002))) ;; move-to bug?
  (setq *laundry-spot* (make-cascoords :pos #f(3326.358 1349.163 -0.854) :rpy #f(1.544 0.004 0.002)))
  (send *room610* :assoc *laundry-spot*)
  (setq *pick-broom-spot* (make-cascoords :pos #f(3269.927 1351.466 0) :rpy (list pi/2 0 0)))
  (send *room610* :assoc *pick-broom-spot*)
  (setq *sink-spot* (make-cascoords :pos #f(7300 200 0) :rpy (list 0 0 0)))
  (send *room610* :assoc *sink-spot*)
  (setq *sweep-spot* (make-cascoords :pos #f(3550 -300 0) :rpy (list (deg2rad -30) 0 0)))
  (send *room610* :assoc *sweep-spot*)

  ;;after move-to, save the pos
  (setq *moved-chair-spot* nil)


  (if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init :objects (send *room610* :objects))))
  (if (not (boundp '*tfl*)) (setq *tfl* (instance transform-listener :init)))

  (setq *world-to-610* (send *tfl* :lookup-transform "/world" *room610-origin* (ros::time 0)))

  (change-inflation-range 0.1)
  (if (not (boundp '*irtviewer*)) (clear-costmap))
  )


(defmethod room610-scene
  (:spot-tray-spot () *tray-spot*)
  (:spot-chair-spot () *chair-spot*)
  (:spot-chair-spot2 () *chair-spot2*)
  (:spot-laundry-spot () *laundry-spot*)
  (:spot-pick-broom-spot () *pick-broom-spot*)
  (:spot-sink-spot () *sink-spot*)
  (:spot-sweep-spot () *sweep-spot*)
  (:spots ()
          (list *tray-spot* *chair-spot* *laundry-spot* *pick-broom-spot* *sink-spot* *sweep-spot*)))

(defun move-to-sink ()
  (warn "move-to-sink~%")
  (use-tilt-laser-obstacle-cloud nil)
  (clear-costmap)
  (send *pr2* :move-to (send *room610* :spot-sink-spot) :world)
  (send *ri* :move-to (send *room610* :spot-sink-spot) :frame-id *room610-origin*)
  (use-tilt-laser-obstacle-cloud t)
  )

(defun move-to-chair ()
  (warn "move-to-chair~%")
  (pr2-tuckarm-pose)
;  (use-tilt-laser-obstacle-cloud nil)
  (clear-costmap)
  (send *pr2* :move-to (send *room610* :spot-chair-spot) :world)
  (send *ri* :move-to (send *room610* :spot-chair-spot) :frame-id *room610-origin*)
;  (use-tilt-laser-obstacle-cloud t)
  (pr2-reset-pose)
  )

(defun move-to-chair2 ()
  (warn "move-to-chair2~%")
  (use-tilt-laser-obstacle-cloud nil)
  (clear-costmap)
  (send *pr2* :move-to (send *room610* :spot-chair-spot2) :world)
  (send *ri* :move-to (send *room610* :spot-chair-spot2) :frame-id *room610-origin*)
  (use-tilt-laser-obstacle-cloud t)
  )

(defun move-to-back-chair ()
  (warn "move-to-chair2~%")
  (use-tilt-laser-obstacle-cloud nil)
  (clear-costmap)
  (if (*moved-chair-spot*)
    (progn
      (send *pr2* :move-to *moved-chair-spot* :world)
      (send *ri* :move-to *moved-chair-spot* :frame-id *room610-origin*)
    ))
  (use-tilt-laser-obstacle-cloud t)
)


(defun move-to-laund ()
  (setq ima (send *ri* :state :worldcoords "/eng8/6f/610"))
  (setq diff (v- (send *laundry-spot* :pos) (send ima :pos)))
  (send *laundry* :translate diff (send *room610* :worldcoords))
  
  )

(defun relocate-pr2 ()
  (setq ima (send *ri* :state :worldcoords "/eng8/6f/610"))
  (send *pr2* :move-to ima :world))

(defun mtl ()
  (setq ima (send *ri* :state :worldcoords "/eng8/6f/610"))
  (send *pr2* :move-to ima :world)
)

(defun test ()
  (init)
  (mtl)
  (put-cloth)
)

(defun move-to-laundry ()
  (warn "move-to-laundry~%")
  (let (current-ri-coordsmou-chotto-pos mou-chotto-local-pos mou-chotto-roll)
    (use-tilt-laser-obstacle-cloud nil)
    (send *pr2* :move-to (send *room610* :spot-laundry-spot) :world)
    (send *ri* :move-to (send *room610* :spot-laundry-spot) :frame-id *room610-origin*)
#|    (cond 
     (*world-to-610*
      (send *ri* :move-to (send *room610* :spot-laundry-spot) :frame-id *room610-origin*)
      (setq current-ri-coords (send (send *world-to-610* :worldcoords) :transformation (send *ri* :state :worldcoords)))
      (setq mou-chotto-pos (v- (send (send *room610* :spot-laundry-spot) :pos) (send current-ri-coords :pos)))
      (setq mou-chotto-local-pos (transform (inverse-matrix (send (send *ri* :state :worldcoords) :worldrot)) mou-chotto-pos))
      (setq mou-chotto-roll
            (-
             (elt (elt (send (send *room610* :spot-laundry-spot) :rpy-angle) 0) 0)
             (elt (elt (send current-ri-coords :rpy-angle) 0) 0)))
      (send *ri* :go-pos-unsafe (/ (elt mou-chotto-local-pos 0) 1000) (/ (elt mou-chotto-local-pos 1) 1000) (rad2deg mou-chotto-roll)))
     (t
      (send *ri* :move-to (send *pr2* :worldcoords))
      ))
|#
    (use-tilt-laser-obstacle-cloud t)
    ))

(defun move-to-broom ()
  (warn "move-to-broom~%")
  (let (current-ri-coords mou-chotto-pos mou-chotto-local-pos mou-chotto-roll)
    (use-tilt-laser-obstacle-cloud nil)
    (send *pr2* :move-to (send *room610* :spot-pick-broom-spot) :world)
    (send *ri* :move-to (send *room610* :spot-pick-broom-spot) :frame-id *room610-origin*)
#|
    (cond
     (*world-to-610*
      (setq current-ri-coords (send (send *world-to-610* :worldcoords) :transformation (send *ri* :state :worldcoords)))
      (setq mou-chotto-pos (v- (send (send *room610* :spot-pick-broom-spot) :pos) (send current-ri-coords :pos)))
      (setq mou-chotto-local-pos (transform (inverse-matrix (send (send *ri* :state :worldcoords) :worldrot)) mou-chotto-pos))
      (setq mou-chotto-roll
            (- 
             (elt (elt (send (send *room610* :spot-pick-broom-spot) :rpy-angle) 0) 0)
             (elt (elt (send current-ri-coords :rpy-angle) 0) 0)))
      (send *ri* :go-pos-unsafe (/ (elt mou-chotto-local-pos 0) 1000) (/ (elt mou-chotto-local-pos 1) 1000) (rad2deg mou-chotto-roll)))
     (t
      (send *ri* :move-to (send *pr2* :worldcoords))
      ))
|#
    (use-tilt-laser-obstacle-cloud t)
    ))


(defun rarm-tuck ()
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(send *pr2* :rarm :wrist-r  :joint-angle 19.9)
(send *pr2* :rarm :wrist-p  :joint-angle -6)
(send *pr2* :rarm :elbow-r  :joint-angle 69.8225)
(send *pr2* :rarm :elbow-p  :joint-angle -90.0744)
(send *pr2* :rarm :shoulder-r  :joint-angle -104.746)
(send *pr2* :rarm :shoulder-p  :joint-angle 69.8)
(send *pr2* :rarm :collar-y  :joint-angle -5.05)
(send *ri* :angle-vector (send *pr2* :angle-vector))
(send *ri* :wait-interpolation)
)

(defun move-to-sweep ()
  (warn "move-to-sweep~%")
;  (rarm-tuck)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (let ((larm-end-coords (send *pr2* :larm :end-coords)))
    (send *pr2* :rarm :inverse-kinematics
          (send larm-end-coords :translate #f(0 0 -300) :world)
          :look-at-target t
          :rotation-axis :z
          :debug-view nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

  (send *pr2* :rarm :inverse-kinematics
          (send larm-end-coords :translate #f(30 0 0) :world)
          :look-at-target t
          :rotation-axis :z
          :debug-view nil))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))

  (send *ri* :start-grasp)

  (use-tilt-laser-obstacle-cloud nil)
  (send *pr2* :move-to (send *room610* :spot-sweep-spot) :world)
  (send *ri* :move-to (send *room610* :spot-sweep-spot) :frame-id *room610-origin*)
  (use-tilt-laser-obstacle-cloud t)
)

(defun pass-broom-to-right-hand ()
  (warn "pass-broom-to-right-hand~%"))

(defun move-to-sweep-floor ()
  (warn "move-to-sweep-floor~%")
  (send *ri* :go-pos-unsafe -0.15 0)
  (send *pr2* :translate (send *pr2* :transform-vector #f(-150 0 0)) :world))

(defun sweep-floor ()
  (warn "sweep-floor (furuta)~%")
  (furuta-sweep)
  )


(defun demo ()
  (pr2-reset-pose)

  (move-to-table)
  (pick-tray)
  ;;
  (move-to-sink)
  (place-tray)
  ;;
  (move-to-chair)
  (pick-cloth)
  ;;
  (move-to-laundry)
  (put-cloth)
  ;;
  (move-to-broom)
  (pick-broom)
  ;;
  (move-to-chair2)
  (move-chair)
  ;;
  (move-to-sweep)
  (sweep-under-table)
  (move-to-sweep-floor)
  (sweep-floor)
  (use-tilt-laser-obstacle-cloud t)
  )

(warn "type (demo) to start~%")
(warn "type (setup) to revert all object location~%")

(setup)
