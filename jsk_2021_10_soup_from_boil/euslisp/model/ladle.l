(defclass ladle-object
  :super cascaded-link
  :slots (
      handle0
      attention0
      attention1
         ))

(defmethod ladle-object
  (:init
   (&rest args &key (name "ladle") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5)
     (send-super* :init :name name args)

     ;; definition of link
     (setq b1 (make-sphere 47))
     (setq b2 (make-sphere 45))
     (setq b1 (body- b1 b2))

     (setq b3 (make-cube 100 100 100))
     (send b3 :locate #f(0 0 51))

     (setq b1 (body- b1 b3))
     
     ;; (send b1 :locate #f(25 0 -200))
     ;; (send b1 :rotate (deg2rad 30) :y)

     ;; (setq b4 (make-cube 13 18 190))
     ;; (send b4 :locate #f(0 0 -30))

     ;; (setq b5 (make-cube 5 10 55))
     ;; (send b5 :locate #f(-7 0 -151))
     ;; (send b5 :rotate (deg2rad 15) :y)

     (send b1 :locate #f(200 0 25))
     (send b1 :rotate (deg2rad -90) :y)
     (send b1 :rotate (deg2rad 30) :y)

     (setq b4 (make-cube 190 18 13))
     (send b4 :locate #f(30 0 0))

     (setq b5 (make-cube 55 10 5))
     (send b5 :locate #f(151 0 -7))
     (send b5 :rotate (deg2rad 15) :y)

     (send b1 :assoc b4)
     (send b1 :assoc b5)     
          
     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1 b4 b5) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))


     ;; definition of assoc
     (send self :assoc link1)

     ;; definition of end-coords

     ;; definition of joint

     ;; definition of :handle
     (setq handle0 (make-cascoords :pos (float-vector 0.0 0.0 0.0) :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0)) :name "handle0"))
     (send link1 :assoc handle0)
     
     ;; definition of :attention
     (setq attention0 (make-cascoords :pos (float-vector 220 0 60) :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0)) :name "attention0"))
     (send link1 :assoc attention0)
     (setq attention1 (make-cascoords :pos (float-vector 220 0 60) :rot #2f((0.5 0.0 -0.866025) (0.0 1.0 0.0) (0.866025 0.0 0.5)) :name "attention1"))
     (send link1 :assoc attention1)
     
     ;; init-ending 
     (setq links (list link1))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)

     self))

  (:handle (&rest args) (forward-message-to-all (list  handle0) args))
  (:handle-handle0 (&rest args) (forward-message-to handle0 args))
  (:attention (&rest args) (forward-message-to-all (list attention0 attention1) args))
  (:attention-attention0 (&rest args) (forward-message-to-all attention0 args))
  (:attention-attention1 (&rest args) (forward-message-to-all attention1 args))
  )

(defun ladle (&rest args) (instance* ladle-object :init args))
