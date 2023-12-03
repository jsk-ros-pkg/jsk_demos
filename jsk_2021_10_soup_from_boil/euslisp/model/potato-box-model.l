(defclass potato-object
  :super bodyset-link
  :slots (width
      length
      height
      ))

(defmethod potato-object
  (:init
   (&rest args &key (w 90) (l 60) (h 60) &allow-other-keys)
   (let (b)
     (setq width w)
     (setq length l)
     (setq height h)
     (setq b (make-cube w l h))
     (send b :set-color :saddlebrown)
     (send-super* :init (make-cascoords) :bodies (list b) args)))
  (:print-info
   ()
   (format t "width ~A, length ~A, height ~A~%" width length height))
  (:change-color
   ()
   (let (b r)
     (setq b (make-cub w l h))
     (send b :newcoords (send self :copy-worldcoords))
     (send b :set-color :saddlebrown)
     (push b r)
     (setq geo::bodies r)))
  (:cut
   ()
   (let (b1 b2 b3 b4 b5 b6 b7 r)
     (setq p1 (make-cube width length height))
     (send p1 :set-color :saddlebrown)

     (setq *cut-point-1* (make-cascoords :pos (float-vector 0 0 (/ height -2))))
     (send *cut-point-1* :rotate (deg2rad 90) :z)
     (send p1 :assoc *cut-point-1*)
     
     (send p1 :newcoords (send self :copy-worldcoords))
     (send self :assoc p1)
     
     (push p1 r)
     (setq geo::bodies r))
   )
  )

(defun potato (&rest args) (instance* potato-object :init args))
