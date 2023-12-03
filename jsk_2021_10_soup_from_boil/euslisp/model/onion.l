(defun z-mul (point num)
  (float-vector (elt point 0) (elt point 1) (* (elt point 2) num)))

(defclass onion-object
  :super cascaded-link
  :slots (
         ))

(defmethod onion-object
  (:init
   (&rest args &key (name "onion") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5)
     (send-super* :init :name name args)

     ;; definition of link
     (setq b1 (make-sphere 45))
     (setq v-list (list (z-mul (car (send b1 :vertices)) 0.78)))
     (dolist (x (cdr (send b1 :vertices))) (setq v-list (append v-list (list (z-mul x 0.78)))))
     (setq b2 (convex-hull-3d v-list))
     
     (setq b3 (make-cylinder 8 2))
     (setq v-list (list (car (send b3 :vertices))))
     (dolist (x (cdr (send b3 :vertices))) (if (< (elt x 2) 1) (setq v-list (append v-list (list x)))))
     
     (setq b4 (make-cone #f(0 0 20) v-list))
     (send b4 :translate #f(0 0 28))

     (send b2 :set-color :peru)
     (send b4 :set-color :peru)

     (send b2 :assoc b4)
     
     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b2 b4) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))


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

(defun onion (&rest args) (instance* onion-object :init args))
