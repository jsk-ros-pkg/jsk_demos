(defclass peeler-object
  :super cascaded-link
  :slots (
          handle0
          handle1
          attention0
          ))

(defmethod peeler-object
  (:init
   (&rest args &key (name "peeler") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5)
     (send-super* :init :name name args)
     
     ;; definition of link
     (setq b1 (make-cube 100 12 12))

     (setq b2 (make-cylinder 30 5))
     (setq b3 (make-cylinder 25 5))
     (setq b2 (body- b2 b3))
     (setq b4 (make-cube 100 100 20))
     (send b4 :locate #f(51 0 0))
     (setq b2 (body- b2 b4))

     (setq b5 (make-cube 5 50 2))
     (send b5 :locate #f(0 0 2))
     (send b5 :set-color :white)

     (send b2 :assoc b5)
     (send b2 :locate #f(80 0 0))

     (send b1 :assoc b2)
     (send b2 :set-color :red)
     (send b1 :set-color :red)
     
     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1 b2 b5) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))


     ;; definition of assoc
     (send self :assoc link1)

     ;; definition of end-coords

     ;; definition of joint

     ;; definition of :handle
     (setq handle0 (make-cascoords :pos #f(0 0 0) :name "handle0"))
     (send link1 :assoc handle0)
     (setq handle1 (make-cascoords :pos #f(0 0 0) :name "handle1"))
     (send handle1 :rotate (deg2rad 90) :x)
     (send link1 :assoc handle1)
     
     ;; definition of :attention
     (setq attention0 (make-cascoords :pos #f(80 0 0) :name "attention0"))
     (send link1 :assoc attention0)

     ;; init-ending 
     (setq links (list link1))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)

     self))
  (:handle (&rest args) (forward-message-to-all (list handle0 handle1) args))
  (:handle-handle0 (&rest args) (forward-message-to handle0 args))
  (:handle-handle1 (&rest args) (forward-message-to handle1 args))
  (:attention (&rest args) (forward-message-to-all (list  attention0) args))
  (:attention-attention0 (&rest args) (forward-message-to attention0 args))
  )

(defun peeler (&rest args) (instance* peeler-object :init args))
