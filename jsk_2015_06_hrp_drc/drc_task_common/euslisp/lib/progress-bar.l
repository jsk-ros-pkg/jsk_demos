(defclass progress-bar
  :super propertied-object
  :slots (max-num))

(defmethod progress-bar
  (:init (amax-num)
    (setq max-num amax-num))
  
  )
