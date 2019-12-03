(defun make-body
    (lx ly lz w)
  (setq m-body-cube (make-cube lx ly lz))
  (setq m-body-in (make-cube (- lx w) (- ly w) (- lz w)))
  (send m-body-in :move-to (make-coords :pos (float-vector (- w) 0 0)) :world)
  (setq m-body (body- m-body-cube m-body-in))
  (send m-body :move-to (make-coords :pos (float-vector (/ lx 2.0) 0 (/ lz 2.0)) :rpy (float-vector 0 0 0)) :world)
  (send m-body :set-color :gray)
  (send m-body :locate (float-vector 0 (/ ly -2) 0))
  m-body)

(defun make-door
    (ly lz wd)
  (setq m-door-cube (make-cube (- ly 80) wd lz))
  (setq m-door-cube-right (make-cube 0 wd lz))
  (setq m-door m-door-cube)
  (send m-door :move-to (make-coords :pos (float-vector 0 (/ -80 2) (/ lz 2.0)) :rpy (float-vector (deg2rad 90) 0 0)) :world)
  (send m-door :set-color :white)
  (send m-door :locate (float-vector (/ ly 2) 0 0))
  m-door)

(defun make-handle
    (ly)
  (setq m-handle (make-cube 40 40 40))
  (send m-handle :translate #f(100 0 0))
  (send m-handle :move-to (make-coords :pos (float-vector 0 (- ly 80) 0) :rpy (float-vector (deg2rad 90) 0 0)) :world)
  (send m-handle :locate (float-vector 0 0 0))
  m-handle)

(defclass microwave-model
    :super cascaded-link
  :slots (m-body m-door m-joint m-handle))

(defmethod microwave-model
    (:init
     ()
     (let ((lx 320)
           (ly 454)
           (lz 255)
           (wd 34)
           (w 30))

       (send-super :init)

       (setq m-body (instance bodyset-link :init (make-cascoords) :bodies (list (make-body lx ly lz w)) :locate (float-vector 0 (/ ly 2) 0)))
       ;;(send m-body :locate (float-vector 0 (/ ly 2) 0))

       (setq m-door (instance bodyset-link :init (make-cascoords) :bodies (list (make-door ly lz wd)) :locate #f(100 0 0)))
       ;;(send m-door :locate (float-vector 0 0 0))

       (setq m-handle (instance bodyset-link :init (make-cascoords) :bodies (list (make-handle ly)) :locate (float-vector 0  ly 0)))
       ;;(send m-handle :locate (float-vector 0 (/ ly 2) 0))

       (setq m-joint (instance rotational-joint :init :parent-link m-body :child-link m-door :axis :z :min -180 :max 360))

       (setq links (list m-body m-door))
       (setq joint-list (list m-joint))

       (send self :assoc m-body)
       (send m-body :assoc m-door)
       (send m-door :assoc m-handle)

       (send self :init-ending)
       self))

  (:get-handle () m-handle)

  (:m-joint (&rest args) (forward-message-to m-joint args))
  )

(defun microwave-init
    ()
  (setq inst (instance microwave-model :init))
  (send inst :angle-vector (float-vector 180))
  inst)
