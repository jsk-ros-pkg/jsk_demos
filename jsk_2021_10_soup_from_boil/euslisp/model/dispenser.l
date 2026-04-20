(defclass dispenser-object
  :super cascaded-link
  :slots (
      handle0
      handle1
      attention0
      ))

(defmethod dispenser-object
  (:init
   (&rest args &key (name "dispenser") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5 b6 b7 b8)
     (send-super* :init :name name args)
     
     ;; definition of link
     (setq b1 (make-cylinder 27 155))
     (setq b2 (make-cone #f(0 0 50) (list #f(10 0 0) #f(0 10 0) #f(-10 0 0) #f(0 -10 0))))
     (send b2 :translate #f(0 0 154))
     
     (send b1 :set-color :White)
     (send b2 :set-color :White)
     
     (send b1 :assoc b2)
     
     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1 b2) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))
     
     
     ;; definition of assoc
     (send self :assoc link1)
     
     ;; definition of end-coords
     
     ;; definition of joint
     
     ;; definition of :handle
     (setq handle0 (make-cascoords :pos #f(0 0 80) :rpy #f(0 0 0) :name "handle0"))
     (send link1 :assoc handle0)

     (setq handle1 (make-cascoords :pos #f(0 0 100) :rpy #f(0 0 0) :name "handle1"))
     (send link1 :assoc handle1)

     ;; definition of :attention
     (setq attention0 (make-cascoords :pos #f(0 0 200) :rpy #f(1.571 -1.571 0.0) :name "attention0"))
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
  (:attention (&rest args) (forward-message-to-all (list attention0) args))
  (:attention-attention0 (&rest args) (forward-message-to attention0 args))
  )

(defun dispenser (&rest args) (instance* dispenser-object :init args))
