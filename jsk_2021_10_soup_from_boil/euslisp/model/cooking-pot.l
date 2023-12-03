(defclass cooking-pot-object
  :super cascaded-link
  :slots (
         ))

(defmethod cooking-pot-object
  (:init
   (&rest args &key (name "cooking-pot") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 bc1 bc2)
     (send-super* :init :name name args)

     ;; definition of link
     (setq bc (make-solid-of-revolution (list #f(0 0 10) #f(100 0 10) #f( 120 0 110) #f(130 0 110) #f(110 0 5) #f(105 0 0) #f(0 0 0))))  
     (send bc :locate #f(0 0 5))
     (send bc :set-color :red)

     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list bc) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))


     ;; definition of assoc
     (send self :assoc link1)

     ;; definition of end-coords

     ;; definition of joint

     ;; definition of :handle

     ;; init-ending 
     (setq links (list link1))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)

     self))

  )

(defun cooking-pot (&rest args) (instance* cooking-pot-object :init args))
