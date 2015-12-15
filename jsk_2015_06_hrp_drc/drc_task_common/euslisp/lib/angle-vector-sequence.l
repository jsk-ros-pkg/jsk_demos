(defclass angle-vector-sequence
  :super propertied-object
  :slots (angle-vectors))

(defmethod angle-vector-sequence
  (:init ()
    )
  (:add (av)
    (setq angle-vectors (append angle-vectors (list av))))
  )
