(defclass spatula-object
  :super cascaded-link
  :slots (
      handle0
      attention0
      attention1
         ))

(defmethod spatula-object
  (:init
   (&rest args &key (name "spatula") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2)
     (send-super* :init :name name args)

     ;; definition of link
     (setq b1 (make-prism (list #f(60 40 0) #f(90 -40 0) #f(0 -35 0) #f(-45 -12 0) #f(-45 12 0) #f(0 35 0)) 8))
     (setq b2 (make-cube 180 15 8))
     
     (send b2 :locate #f(45 0 0))
     (send b1 :locate #f(180 0 -4))

     (send b1 :set-color :BurlyWood)
     (send b2 :set-color :BurlyWood)
     (send b1 :assoc b2)

     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1 b2) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))


     ;; definition of assoc
     (send self :assoc link1)

     ;; definition of end-coords

     ;; definition of joint

     ;; definition of :handle
     (setq handle0 (make-cascoords :pos (float-vector 0.0 0.0 0.0) :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0)) :name "handle0"))
     (send link1 :assoc handle0)

     ;; definition of :attention
     (setq attention0 (make-cascoords :pos (float-vector 260 0 0) :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0)) :name "attention0"))
     (send link1 :assoc attention0)
     (setq attention1 (make-cascoords :pos (float-vector 260 0 0) :rot #2f((0.965926 -0.258819 0.0) (0.258819 0.965926 0.0) (0.0 0.0 1.0)) :name "attention1"))
     (send link1 :assoc attention1)


     ;; init-ending 
     (setq links (list link1))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)

     self))

  (:handle (&rest args) (forward-message-to-all (list handle0) args))
  (:handle-handle0 (&rest args) (forward-message-to handle0 args))
  (:attention (&rest args) (forward-message-to-all (list attention0 attention1) args))
  (:attention-attention0 (&rest args) (forward-message-to-all attention0 args))
  (:attention-attention1 (&rest args) (forward-message-to-all attention1 args))
  )

(defun spatula (&rest args) (instance* spatula-object :init args))
