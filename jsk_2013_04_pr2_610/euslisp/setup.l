#!/usr/bin/env roseus

(load "models/room610-scene.l")
(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")

(if (not (boundp '*room610*)) (room610))
(if (not (boundp '*pr2*)) (pr2))
(if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init :objects (send *room610* :objects))))

(objects (list *pr2*))

(setq *room610-origin* "/eng8/6f/room610")
(setq *table-coords* (make-coords :pos #f(4700 1000 0) :rpy (float-vector -pi/2 0 0)))
(setq *chair-coords* (make-coords :pos #f(3800 0 0)))

(load "package://jsk_2013_04_pr2_610/euslisp/pick-broom.l")
(load "package://jsk_2013_04_pr2_610/euslisp/open-laundary.l")


(defun move-to-table ()
  (warn "move-to-table~%")
  (pr2-tuckarm-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :move-to *table-coords* :frame-id *room610-origin*)
  )

(defun pick-tray ()
  (warn "pick-tray (tanaka)~%")
  )

(defun move-to-sink ()
  (warn "move-to-sink~%")
  )

(defun place-tray ()
  (warn "place-tray (chan)~%")
  )

(defun move-to-chair ()
  (warn "move-to-chair~%")
  (pr2-tuckarm-pose)
  (send *ri* :move-to *chair-coords* :frame-id *room610-origin*)
  )

(defun pick-cloth ()
  (warn "pick-cloth (iwaishi)~%")
  )

(defun move-to-laundary ()
  (warn "move-to-laundary~%")
  )

(defun put-cloth ()
  (warn "put-cloth (kuroiwa)~%")
  )

(defun close-laundry-door ()
  (warn "close-laundry-door (kuroiwa)~%")
  )


(defun move-chair ()
  (warn "move-chair (inagaki)~%")
  ;; show chair on objects
  ;; TODO update chair position using perception pipeline,
  ;;   load and instantiate chair model at setup function, 
  ;;   start/stop perception piplen in move-chair function
  (load "models/red-chair0-object.l") ;; need to use karimoku chair
  (setq *chair* (red-chair0))
  (send *chair* :move-to (make-coords :pos #f(800 0 0) :rpy (float-vector pi/2 0 0)) :world)
  (objects (list *pr2* *chair*))
  ;; use handle to manipulate
  (send *pr2* :reset-pose) ;; to solve ik, do not  send to *ri*
  (send *pr2* :inverse-kinematics (send *chair* :handle)
        :link-list (list
                      (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                      (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
        :move-target (list (send *pr2* :larm :end-coords) (send *pr2* :rarm :end-coords))
        :look-at-target t
        )
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :start-grasp)
  ;; backword
  (send *ri* :go-pos-unsafe -0.5 0 30)
  (send *ri* :stop-grasp)
  )

(defun sweep-under-table ()
  (warn "sweep-under-table (yukizaki)~%")
  )

(defun pass-broom-to-right-hand ()
  (warn "pass-broom-to-right-hand~%"))

(defun sweep-floor ()
  (warn "sweep-floor (furuta)~%")
  )


(defun demo ()
  (move-to-table)
  (pick-tray)
  (move-to-sink)
  (place-tray)
  (move-to-chair)
  (pick-cloth)
  (move-to-laundary)
  (open-laundary-door)
  (put-cloth)
  (close-laundry-door)
  (pick-broom)
  (move-chair)
  (sweep-under-table)
  (sweep-floor)
  )

(warn "type (demo) to start~%")
