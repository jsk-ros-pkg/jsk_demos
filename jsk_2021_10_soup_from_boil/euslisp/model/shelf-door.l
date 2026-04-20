(defclass shelf-door-object
  :super cascaded-link
  :slots (
      handle0
         ))

(defmethod shelf-door-object
  (:init
   (&rest args &key (name "shelf-door") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1
     b1
     b2
     (sy 280)
     (sz 60)
     )
     (send-super* :init :name name args)

     ;; definition of link
     (setq b1 (make-cube 10 sy sz))     
     (setq b2 (make-cube 10 10 30))

     (send b1 :locate (float-vector 0 (/ sy 2) 0))
     (send b2 :locate (float-vector -10 (- sy 20) 0))

     (send b1 :set-color :blue)
     (send b2 :set-color :yellow)
     (send b1 :assoc b2)
     

     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1 b2) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))

     ;; link0
     (setq b3 (make-cube 100 10 sz))
     (send b3 :locate #f(50 0 0))
     (send b3 :set-color :blue)
     
     (setq link0 (instance bodyset-link :init (make-cascoords) :bodies (list b3) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))
     
     ;; definition of assoc
     (send self :assoc link0)
     (send link0 :assoc link1)

     ;; definition of end-coords
     
     ;; definition of joint
     (setq joint1 (instance rotational-joint :init :parent-link link0 :child-link link1 :axis :z :min 0 :max 180))
     
     ;; definition of :handle
     (setq handle0 (make-cascoords :pos (float-vector 0.0 0.0 0.0) :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0)) :name "handle0"))
     (send handle0 :locate (float-vector -15 (- sy 20) 0))
     (send link1 :assoc handle0)

     ;; definition of :attention
     

     ;; init-ending 
     (setq links (list link0 link1))
     (setq joint-list (list joint1))
     
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)

     self))

  (:handle (&rest args) (forward-message-to-all (list  handle0) args))
  (:handle-handle0 (&rest args) (forward-message-to handle0 args))
  )

(defun shelf-door (&rest args) (instance* shelf-door-object :init args))
