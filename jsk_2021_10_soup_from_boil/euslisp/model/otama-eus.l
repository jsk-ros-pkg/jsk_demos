(defclass otama-eus-object
  :super cascaded-link
  :slots (
      ))

(defmethod otama-eus-object
  (:init
   (&rest args &key (name "otama-eus") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2)
     (send-super* :init :name name args)

     ;; definition of link
     (setq b1 (make-sphere 47))
     (setq b2 (make-sphere 45))
     (setq b1 (body- b1 b2))

     (setq b3 (make-cube 100 100 100))
     (send b3 :locate #f(0 0 51))

     (setq b1 (body- b1 b3))
     (send b1 :rotate (deg2rad 30) :y)

     (setq b4 (make-cube 10 20 240))
     (send b4 :locate #f(-37 0 135))
          
     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1 b4) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))


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

(defun otama-eus (&rest args) (instance* otama-eus-object :init args))
