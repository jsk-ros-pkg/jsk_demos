(defclass bowl-4-object
  :super cascaded-link
  :slots (
      handle0
      attention0
      ))

(defmethod bowl-4-object
  (:init
   (&rest args &key (name "bowl-4") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5)
     (send-super* :init :name name args)

     ;; definition of link
     (setq b1 (make-sphere 112))
     (setq b2 (make-cube 1000 1000 200))
     (send b2 :locate #f(0 0 101))
     (setq b3 (make-sphere 107))
     (setq b4 (make-cylinder 58 5))
     (setq b5 (make-cube 1000 1000 40))
     (send b5 :locate #f(0 0 -110))

     (setq b1 (body- b1 b2))
     (setq b1 (body- b1 b3))
     (setq b1 (body- b1 b5))
     
     (send b1 :locate #f(0 0 90))
     (send b4 :locate #f(0 0 0))

     (send b1 :set-color :Silver)
     (send b4 :set-color :Silver)
     
     (send b1 :assoc b4)
     
     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1 b4) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))


     ;; definition of assoc
     (send self :assoc link1)

     ;; definition of end-coords

     ;; definition of joint
     
     ;; definition of :handle
     (setq handle0 (make-cascoords :pos (float-vector 100 0 80) :rpy #f(-3.141 1.047 1.571) :name "handle0"))
     (send link1 :assoc handle0)
     
     ;; definition of :attention
     (setq attention0 (make-cascoords :pos (float-vector -105 0 85) :rpy (float-vector 3.142 -1.571  0) :name "attention0"))
     (send link1 :assoc attention0)
     
     ;; init-ending 
     (setq links (list link1))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)

     self))

  (:handle (&rest args) (forward-message-to-all (list handle0) args))
  (:handle-handle0 (&rest args) (forward-message-to handle0 args))
  (:attention (&rest args) (forward-message-to-all (list attention0) args))
  (:attention-attention0 (&rest args) (forward-message-to-all attention0 args))
  )

(defun bowl-4 (&rest args) (instance* bowl-4-object :init args))
