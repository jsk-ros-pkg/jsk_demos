(defclass solt-object
  :super cascaded-link
  :slots (
      handle0
      attention0
      ))

(defmethod solt-object
  (:init
   (&rest args &key (name "solt") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5 b6 b7 b8)
     (send-super* :init :name name args)
     
     ;; definition of link
     (setq b1 (make-cube 40 40 60))
     (setq b2 (make-cylinder 17.5 20))
     (setq b3 (make-cylinder 20 20))
     
     (send b1 :locate #f(0 0 30))
     (send b2 :locate #f(0 0 60))
     (send b3 :locate #f(0 0 80))
     
     (send b1 :set-color :LightCyan)
     (send b2 :set-color :LightCyan)
     (send b3 :set-color :silver)
     
     (send b1 :assoc b2)
     (send b1 :assoc b3)
     
     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1 b2 b3) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))


     ;; definition of assoc
     (send self :assoc link1)

     ;; definition of end-coords

     ;; definition of joint

     ;; definition of :handle
     (setq handle0 (make-cascoords :pos #f(0 0 30) :rpy #f(0 0 0) :name "handle0"))
     (send link1 :assoc handle0)
     
     ;; definition of :attention
     (setq attention0 (make-cascoords :pos #f(0 0 100) :rpy #f(0 -1.571 0) :name "attention0"))
     (send link1 :assoc attention0)
     
     ;; init-ending 
     (setq links (list link1))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)

     self))
  
  (:handle (&rest args) (forward-message-to-all (list handle0) args))
  (:handle-handle0 (&rest args) (forward-message-to handle0 args))
  (:attention (&rest args) (forward-message-to-all (list  attention0) args))
  (:attention-attention0 (&rest args) (forward-message-to attention0 args))
  )

(defun solt (&rest args) (instance* solt-object :init args))
