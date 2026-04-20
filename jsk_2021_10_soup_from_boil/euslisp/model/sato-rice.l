(defclass sato-rice-object
  :super cascaded-link
  :slots (
          handle0
          handle1
          attention0
          ))

(defmethod sato-rice-object
  (:init
   (&rest args &key (name "sato-rice") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5 b6 b7 b8)
     (send-super* :init :name name args)
     
     ;; definition of link
     (setq b1 (make-cube 115 150 35))
     (send b1 :locate #f(0 0 17))
     
     (send b1 :set-color :white)
     
     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))
     
     
     ;; definition of assoc
     (send self :assoc link1)
     
     ;; definition of end-coords
     
     ;; definition of joint
     
     ;; definition of :handle
     (setq handle0 (make-cascoords :pos #f(57 -75 35) :rpy #f(2.356 0.785 1.571) :name "handle0"))
     (send link1 :assoc handle0)
     (setq handle1 (make-cascoords :pos #f(57 75 35) :rpy #f(-2.356 0.785 1.571) :name "handle1"))
     (send link1 :assoc handle1)
     
     ;; definition of :attention
     (setq attention0 (make-cascoords :pos #f(0 75 35) :rpy #f(1.571 0 0) :name "attention0"))
     (send link1 :assoc attention0)
     
     ;; init-ending 
     (setq links (list link1))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)
     
     self))
  
  (:handle (&rest args) (forward-message-to-all (list  handle0 handle1) args))
  (:handle-handle0 (&rest args) (forward-message-to handle0 args))
  (:handle-handle1 (&rest args) (forward-message-to handle1 args))
  (:attention (&rest args) (forward-message-to-all (list  attention0) args))
  (:attention-attention0 (&rest args) (forward-message-to attention0 args))
  )

(defun sato-rice (&rest args) (instance* sato-rice-object :init args))
