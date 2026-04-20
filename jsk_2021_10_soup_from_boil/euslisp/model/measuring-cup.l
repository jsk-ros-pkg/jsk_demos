(defclass measuring-cup-object
  :super cascaded-link
  :slots (handle0
      attention0
      attention1
      ))

(defmethod measuring-cup-object
  (:init
   (&rest args &key (name "measuring-cup") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5 b6 b7 b8)
     (send-super* :init :name name args)

     ;; definition of link
     (setq b1 (make-cylinder 45 110))
     (setq b2 (make-cylinder 43 106))
     (send b2 :locate #f(0 0 4))

     (setq b5 (make-cone #f(0 0 -20) (list #f(15 0 0) #f(0 15 0) #f(-15 0 0) #f(0 -15 0))))
     (setq b6 (make-cone #f(0 0 -18) (list #f(13 0 0) #f(0 13 0) #f(-13 0 0) #f(0 -13 0))))
     (setq b5 (body- b5 b6))
     (setq b7 (make-cube 50 50 50))
     (send b7 :locate #f(0 24 -15))
     (setq b5 (body- b5 b7))
     (send b5 :locate #f(0 -40 110))
     
     (setq b3 (make-cube 20 35 80))
     (setq b4 (make-cube 20 31 72))
     (send b4 :locate #f(0 -2 0))
     (setq b4 (body- b3 b4))
     (send b4 :locate #f(0 17 -40))
     (send b4 :locate #f(0 43 110))

     (setq b1 (body+ b1 b4))
     (setq b1 (body- b1 b2))

     (setq b8 (make-cube 17 10 7))
     (send b8 :locate #f(0 0 -3))
     (send b8 :locate #f(0 -41 111))
     (setq b1 (body- b1 b8))

     (send b1 :set-color :LightCyan)
     (send b5 :set-color :LightCyan)

     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1 b5) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))


     ;; definition of assoc
     (send self :assoc link1)

     ;; definition of end-coords

     ;; definition of joint

     ;; definition of :handle
     (setq handle0 (make-cascoords :pos #f(0 75 70) :rpy #f(3.14 0 0) :name "handle0"))
     (send link1 :assoc handle0)

     ;; definition of :attention
     (setq attention0 (make-cascoords :pos #f(0 -50 110) :name "attention0"))
     (send link1 :assoc attention0)
     (setq attention1 (make-cascoords :pos #f(0 0 110) :rpy #f(-1.57 0 0) :name "attention1"))
     (send link1 :assoc attention1)
    
     ;; init-ending 
     (setq links (list link1))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)

     self))

  (:handle (&rest args) (forward-message-to-all (list  handle0) args))
  (:handle-handle0 (&rest args) (forward-message-to handle0 args))
  (:attention (&rest args) (forward-message-to-all (list  attention0 attention1) args))
  (:attention-attention0 (&rest args) (forward-message-to attention0 args))
  (:attention-attention1 (&rest args) (forward-message-to attention1 args))
  )

(defun measuring-cup (&rest args) (instance* measuring-cup-object :init args))
