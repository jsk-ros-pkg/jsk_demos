(defclass white-dish-object
  :super cascaded-link
  :slots (
          attention0
          ))

(defmethod white-dish-object
  (:init
   (&rest args &key (name "white-dish") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5 b6 b7 b8)
     (send-super* :init :name name args)
     
     ;; definition of link
     (setq b1 (make-cylinder 97 20))
     (setq b2 (make-cylinder 95 16))
     (send b2 :locate #f(0 0 4))
     
     (setq b1 (body- b1 b2))
     
     (send b1 :set-color :white)
     
     ;; (send b1 :assoc b4)
     
     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))
     
     
     ;; definition of assoc
     (send self :assoc link1)
     
     ;; definition of end-coords
     
     ;; definition of joint
     
     ;; definition of :handle
     
     ;; definition of :attention
     (setq attention0 (make-cascoords :pos #f(0 0 80) :rpy #f(0 0 0) :name "attention0"))
     (send link1 :assoc attention0)
     
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

(defun white-dish (&rest args) (instance* white-dish-object :init args))
