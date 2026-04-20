(defun x-mul (point num)
  (float-vector (* (elt point 0) num) (elt point 1) (elt point 2)))

(defclass potato-object
  :super cascaded-link
  :slots (
         ))

(defmethod potato-object
  (:init
   (&rest args &key (name "potato") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5)
     (send-super* :init :name name args)

     ;; definition of link
     (setq b1 (make-sphere 30))
     (setq v-list (list (x-mul (car (send b1 :vertices)) 1.5)))
     (dolist (x (cdr (send b1 :vertices))) (setq v-list (append v-list (list (x-mul x 1.5)))))
     (setq b2 (convex-hull-3d v-list))
     
     (send b2 :set-color :saddlebrown)
     
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

(defun potato (&rest args) (instance* potato-object :init args))
