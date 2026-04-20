(defclass cooking-pot-object
  :super cascaded-link
  :slots (content
      bottom
         ))

(defmethod cooking-pot-object
  (:init
   (&rest args &key (name "cooking-pot") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 bc1 bc2)
     (send-super* :init :name name args)

     ;; definition of link
     (setq bc1 (make-solid-of-revolution (list #f(0 0 10) #f(100 0 10) #f( 120 0 110) #f(130 0 110) #f(110 0 5) #f(105 0 0) #f(0 0 0))))  
     (send bc1 :locate #f(0 0 5))
     (send bc1 :set-color :red)

     (setq bc2 (make-solid-of-revolution (list #f(0 0 110) #f(120 0 110) #f(100 0 10) #f(0 0 10))))
     (setq content bc2)
     (send bc1 :assoc content)

     (setq bc3 (make-solid-of-revolution (list #f(0 0 10) #f(100 0 10) #f(100 0 0) #f(0 0 0))))
     (setq bottom bc3)
     (send bc1 :assoc bottom)  

     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list bc1) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))


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

  (:content (&rest args) (forward-message-to content args))
  (:bottom (&rest args) (forward-message-to bottom args))

  (:water
   (&rest args &key (surface 70))
   (let (b bc1 box)
     (setq bc1 content)
     (send bc1 :copy-worldcoords)
     (setq b (make-bounding-box (send bc1 :vertices)))
     (setq m (v- (send b :maxpoint) (send b :minpoint)))
     (setq box (make-cube (* (elt m 0) 2) (* (elt m 1) 2) (* (elt m 2) 2)))
     (send box :translate (send b :center) :world)

     (send box :translate (float-vector 0 0 (/ (elt m 2) 2)) :world)
     (send box :translate (float-vector 0 0 surface) :world)
     (setq bc1 (body- bc1 box))
     ))
  
  )

(defun cooking-pot (&rest args) (instance* cooking-pot-object :init args))
