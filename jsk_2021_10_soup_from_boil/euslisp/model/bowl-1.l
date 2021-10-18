(defclass bowl-1-object
  :super cascaded-link
  :slots (
      handle0
      attention0
      attention1
      ))

(defmethod bowl-1-object
  (:init
   (&rest args &key (name "bowl-1") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5)
     (send-super* :init :name name args)
     
     ;; definition of link
     (setq b1 (make-sphere 97))
     (setq b2 (make-cube 1000 1000 100))
     (send b2 :locate #f(0 0 51))
     (setq b3 (make-sphere 92))
     (setq b4 (make-cylinder 50 5))
     (setq b5 (make-cube 1000 1000 40))
     (send b5 :locate #f(0 0 -100))

     (setq b1 (body- b1 b2))
     (setq b1 (body- b1 b3))
     (setq b1 (body- b1 b5))
     
     (send b1 :locate #f(0 0 80))
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
     (setq handle0 (make-cascoords :pos (float-vector 80 0 60) :rpy #f(-3.141 1.047 1.571) :name "handle0"))
     (send link1 :assoc handle0)
     
     ;; definition of :attention
     (setq attention0 (make-cascoords :pos (float-vector -90 0 65) :rpy (float-vector 3.142 -1.571  0) :name "attention0"))
     (send link1 :assoc attention0)
     (setq attention1 (make-cascoords :pos (float-vector 0 0 65) :rpy (float-vector 0 0  0) :name "attention1"))
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

(defun bowl-1 (&rest args) (instance* bowl-1-object :init args))
