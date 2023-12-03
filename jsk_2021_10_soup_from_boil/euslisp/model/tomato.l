(defun z-mul (point num)
  (float-vector (elt point 0) (elt point 1) (* (elt point 2) num)))

(defclass tomato-object
  :super cascaded-link
  :slots (
         ))

(defmethod tomato-object
  (:init
   (&rest args &key (name "tomato") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5)
     (send-super* :init :name name args)

     ;; definition of link
     (setq b1 (make-sphere 37))
     (setq v-list (list (z-mul (car (send b1 :vertices)) 0.86)))
     (dolist (x (cdr (send b1 :vertices))) (setq v-list (append v-list (list (z-mul x 0.86)))))
     (setq b2 (convex-hull-3d v-list))

     (send b2 :set-color :red)
     
     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b2) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))


     ;; definition of assoc
     (send self :assoc link1)

     ;; definition of end-coords

     ;; definition of joint

     ;; definition of :handle

     ;; init-ending 
     (setq links (list link1))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)

     self))

  )

(defun tomato (&rest args) (instance* tomato-object :init args))
