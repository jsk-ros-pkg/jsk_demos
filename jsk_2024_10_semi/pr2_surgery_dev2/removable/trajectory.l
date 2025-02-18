(defclass traj
  :super cascaded-coords
  :slots (points))
(defmethod traj
  (:init (&rest args)
	 (send-super* :init args)
    (dotimes (i 10)
      (push
       (make-cube 10 10 10 :pos (float-vector (* (- i 5) 20) 0 0))
       points))
    (dotimes (i (- (length points) 1))
      (send (elt points i) :assoc (elt points (+ 1 i))))
    (send self :assoc (car points))
    self) ;; :init
  (:points () points)
  ) ;; defmethod
