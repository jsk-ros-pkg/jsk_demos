(defun point-mul (point x y z)
  (float-vector (* (elt point 0) x) (* (elt point 1) y) (* (elt point 2) z)))

(defun make-distort (b1 x-scale y-scale)
  (setq v-list (list (point-mul (car (send b1 :vertices)) x-scale y-scale 1)))
  (dolist (x (cdr (send b1 :vertices))) (setq v-list (append v-list (list (point-mul x x-scale y-scale 1)))))
  (setq b2 (convex-hull-3d v-list))
  )

(defclass curry-dish-object
  :super cascaded-link
  :slots (
          attention0
          ))

(defmethod curry-dish-object
  (:init
   (&rest args &key (name "curry-dish") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5 b6 b7 b8)
     (send-super* :init :name name args)
     
     ;; definition of link
     (setq b1 (make-cylinder 75 50))
     (setq b1 (make-distort b1 1 1.73333))
     (setq b2 (make-cylinder 73 46))
     (setq b2 (make-distort b2 1 1.73333))
     (send b2 :locate #f(0 0 4))
     
     (setq b1 (body- b1 b2))
     
     (send b1 :set-color :blue)
     
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

  (:attention (&rest args) (forward-message-to-all (list  attention0) args))
  (:attention-attention0 (&rest args) (forward-message-to attention0 args))
  )

(defun curry-dish (&rest args) (instance* curry-dish-object :init args))
