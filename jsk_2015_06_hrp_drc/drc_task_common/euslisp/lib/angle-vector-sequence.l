(defclass angle-vector-sequence
  :super propertied-object
  :slots (angle-vectors))

(defmethod angle-vector-sequence
  (:init ()
    )
  (:add (av)
    (setq angle-vectors (append angle-vectors (list av))))
  (:filter-look-at-hand (robot-model arm)
    (dotimes (i (length angle-vectors))
      (send robot :angle-vector (elt angle-vectors i))
      (send robot :head :look-at (send robot arm :end-coords :worldpos))
      (setf (elt angle-vectors i) (send robot :angle-vector))
      )
    )
  )
