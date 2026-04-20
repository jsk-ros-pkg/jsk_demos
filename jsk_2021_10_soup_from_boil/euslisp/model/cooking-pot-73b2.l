(defclass cooking-pot-object
  :super cascaded-link
  :slots (
      handle0
      attention0
      content
      bottom
      ))

(defmethod cooking-pot-object
  (:init
   (&rest args &key (name "cooking-pot") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 bc1 bc2 bc3 bc4)
     (send-super* :init :name name args)

     ;; definition of link
     (setq bc1 (make-solid-of-revolution (list #f(0 0 10) #f(70 0 10) #f(95 0 100) #f(100 0 100) #f(73 0 5) #f(72.5 0 0) #f(0 0 0))))  
     (send bc1 :locate #f(0 0 5))
     ;; (send bc1 :set-color :red)

     (setq bc2 (make-solid-of-revolution (list #f(0 0 73) #f(95 0 100) #f(70 0 10) #f(0 0 10))))
     (setq content bc2)
     (send bc1 :assoc content)

     (setq bc3 (make-solid-of-revolution (list #f(0 0 10) #f(70 0 10) #f(70 0 0) #f(0 0 0))))
     (setq bottom bc3)
     (send bc1 :assoc bottom)
     
     (setq bc4 (make-cube 100 35 20))
     (send bc4 :locate #f(-140 0 90))
     (send bc4 :set-color :black)
     (send bc1 :assoc bc4)
     
     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list bc1 bc4) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))
     
     
     ;; definition of assoc
     (send self :assoc link1)

     ;; definition of end-coords
     
     ;; definition of joint
     
     ;; definition of :handle
     (setq handle0 (make-cascoords :pos (float-vector -160 0 90) :rot #2f((0.642788 0.0 0.766044) (0.0 -1.0 0.0) (0.766044 0.0 -0.642788)) :name "handle0"))
     (send link1 :assoc handle0)
     
     ;; definition of :attention
     (setq attention0 (make-cascoords :pos (float-vector 0 0 5) :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0)) :name "attention0"))
     (send link1 :assoc attention0)
     
     ;; init-ending 
     (setq links (list link1))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)
     
     self))
  
  (:handle (&rest args) (forward-message-to-all (list  handle0) args))
  (:handle-handle0 (&rest args) (forward-message-to handle0 args))
  (:attention (&rest args) (forward-message-to-all (list attention0) args))
  (:attention-attention0 (&rest args) (forward-message-to-all attention0 args))
  
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
