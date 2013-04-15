#!/usr/bin/env roseus

(load "models/room610-scene.l")
(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")
(load "package://roseus/euslisp/roseus-utils.l")

(if (not (boundp '*room610*)) (room610))
(if (not (boundp '*pr2*)) (pr2))
(if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init)))

(objects (list *room610* *pr2*))

(defun pick-tray ()
  (warn "pick-tray~%")
  )

(defun move-to-sink ()
  (warn "move-to-sink~%")
  )

(defun place-tray ()
  (warn "place-tray~%")
  )

(defun move-to-chair ()
  (warn "move-to-chair~%")
  )

(defun pick-cloth ()
  (warn "pick-cloth~%")
  )

(defun move-to-laundary ()
  (warn "move-to-laundary~%")
  )

(defun open-laundary-door ()
  (warn "open-laundary-door~%")
  )

(defun put-cloth ()
  (warn "put-cloth~%")
  )

(defun pick-broom ()
  (warn "pick-broom~%")
  (setq prev-angle (send *pr2* :angle-vector)) ;; TODO: 前の動作からの姿勢引き継ぎ
  (send *pr2* :rarm :move-end-pos (float-vector 200 0 0) :world :rotation-axis :z)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (when *irtviewer* (send *irtviewer* :draw-objects))
  (unix:sleep 3)
  (warn "rarm-start-grasp~%")
  (send *ri* :start-grasp :rarm) ;; FIXME: 実機につながってないと関数が終了しない。
  (warn "prev-angle")
  (send *pr2* :angle-vector prev-angle)
  (when *irtviewer* (send *irtviewer* :draw-objects))
  (send *ri* :angle-vector prev-angle 3000)
  )

(defun move-chair ()
  (warn "move-chair~%")
  )

(defun sweep-under-table ()
  (warn "sweep-under-table~%")
  )

(defun sweep-floor ()
  (warn "sweep-floor~%")
  )


(defun demo ()
  (pick-tray)
  (move-to-sink)
  (place-tray)
  (move-to-chair)
  (pick-cloth)
  (move-to-laundary)
  (open-laundary-door)
  (put-cloth)
  (pick-broom)
  (move-chair)
  (sweep-under-table)
  (sweep-floor)
  )

(warn "type (demo) to start~%")
