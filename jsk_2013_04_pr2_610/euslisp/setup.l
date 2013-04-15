(load "models/room610-scene.l")
(load "package://pr2eus/pr2.l")
(if (not (boundp '*room610*)) (room610))
(if (not (boundp '*pr2*)) (pr2))

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

(defun pick-bloom ()
  (warn "pick-bloom~%")
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
  (pick-bloom)
  (move-chair)
  (sweep-under-table)
  (sweep-floor)
  )

(warn "type (demo) to start~%")

