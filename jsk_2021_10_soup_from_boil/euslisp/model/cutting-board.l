(defclass cutting-board-object
  :super cascaded-link
  :slots (
      handle0
      attention0
      attention1
      attention2
      ))

(defmethod cutting-board-object
  (:init
   (&rest args &key (name "cutting-board") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 bc)
     (send-super* :init :name name args)
     
     ;; definition of link
     (setq bc (make-cube 200 350 10))
     (send bc :locate #f(0 0 5))
     (send bc :set-color :white)
     
     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list bc) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))
     
     
     ;; definition of assoc
     (send self :assoc link1)
     
     ;; definition of end-coords
     
     ;; definition of joint
     
     ;; definition of :handle
     (setq handle0 (make-cascoords :pos (float-vector 50 -140 0) :rpy #f(1.882 0.065 1.548) :name "handle0"))
     (send link1 :assoc handle0)
     
     ;; definition of :attention
     (setq attention0 (make-cascoords :pos (float-vector -100 0 0) :rpy (float-vector 0 0 0) :name "attention0"))
     (send link1 :assoc attention0)
     (setq attention1 (make-cascoords :pos (float-vector 100 0 0) :rpy (float-vector 0 0 0) :name "attention1"))
     (send link1 :assoc attention1)
     (setq attention2 (make-cascoords :pos (float-vector 0 175 0) :rpy (float-vector 1.571 0 0) :name "attention1"))
     (send link1 :assoc attention2)
     
     ;; init-ending 
     (setq links (list link1))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)
     
     self))
  
  (:handle (&rest args) (forward-message-to-all (list handle0) args))
  (:handle-handle0 (&rest args) (forward-message-to handle0 args))
  (:attention (&rest args) (forward-message-to-all (list attention0 attention1 attention2) args))
  (:attention-attention0 (&rest args) (forward-message-to-all attention0 args))
  (:attention-attention1 (&rest args) (forward-message-to-all attention1 args))
  (:attention-attention2 (&rest args) (forward-message-to-all attention2 args))
  )

(defun cutting-board (&rest args) (instance* cutting-board-object :init args))
