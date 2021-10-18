(defclass wood-spatula-object
  :super cascaded-link
  :slots (
         ))

(defmethod wood-spatula-object
  (:init
   (&rest args &key (name "wood-spatula") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2)
     (send-super* :init :name name args)

     ;; definition of link
     (setq b1 (make-prism (list #f(50 25 0) #f(50 -25 0) #f(0 -35 0) #f(-45 -12 0) #f(-45 12 0) #f(0 35 0)) 8))
     (setq b2 (make-cube 235 24 8))
     
     (send b2 :locate #f(-162 0 0))
     (send b1 :locate #f(0 0 -4))

     (send b1 :set-color :BurlyWood)
     (send b2 :set-color :BurlyWood)

     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1 b2) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))


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

(defun wood-spatula (&rest args) (instance* wood-spatula-object :init args))
