(defun point-mul (point x y z)
  (float-vector (* (elt point 0) x) (* (elt point 1) y) (* (elt point 2) z)))

(defun make-distorted-cylinder (width length height)
  (let (b1 b2)
    (setq x-scale (/ (float width) (float width)))
    (setq y-scale (/ (float length) (float width)))
    (setq b1 (make-cylinder width height))
    (setq v-list (list (point-mul (car (send b1 :vertices)) x-scale y-scale 1)))
    (dolist (x (cdr (send b1 :vertices))) (setq v-list (append v-list (list (point-mul x x-scale y-scale 1)))))
    (setq b2 (convex-hull-3d v-list))
    )
  )

(defclass measuring-cup-73b2-object
  :super cascaded-link
  :slots (handle0
      handle1
      attention0
      attention1
      ))

(defmethod measuring-cup-73b2-object
  (:init
   (&rest args &key (name "measuring-cup-73b2") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (link1 b1 b2 b3 b4 b5 b6 b7 b8)
     (send-super* :init :name name args)

     ;; definition of link
     (setq b1 (make-distorted-cylinder 42 60 70))
     (setq b2 (make-distorted-cylinder 40 58 65))
     (send b2 :locate #f(0 0 5))
     (setq b1 (body- b1 b2))

     (setq b5 (make-cone #f(0 0 -20) (list #f(15 0 0) #f(0 15 0) #f(-15 0 0) #f(0 -15 0))))
     (setq b6 (make-cone #f(0 0 -18) (list #f(13 0 0) #f(0 13 0) #f(-13 0 0) #f(0 -13 0))))
     (setq b5 (body- b5 b6))
     (setq b7 (make-cube 50 50 50))
     (send b7 :locate #f(0 24 -15))
     (setq b5 (body- b5 b7))
     (send b5 :locate #f(0 -55 70))
     
     (setq b3 (make-cube 23 35 10))
     (send b3 :locate #f(0 72 65))
     (setq b4 (make-cube 23 62 15))
     (send b4 :locate #f(0 100 38))
     (send b4 :rotate (deg2rad -77) :x :world)

     (setq b8 (make-cube 17 10 7))
     (send b8 :locate #f(0 0 -3))
     (send b8 :locate #f(0 -41 111))
     (setq b1 (body- b1 b8))

     (send b1 :set-color :LightCyan)
     (send b3 :set-color :LightCyan)
     (send b4 :set-color :LightCyan)
     (send b5 :set-color :LightCyan)

     (send b1 :assoc b5)
     (send b1 :assoc b3)
     (send b1 :assoc b4)

     (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list b1 b5 b3 b4) :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))


     ;; definition of assoc
     (send self :assoc link1)

     ;; definition of end-coords

     ;; definition of joint

     ;; definition of :handle
     (setq handle0 (make-cascoords :pos #f(0 100 40) :rpy #f(-1.571 0.227 0) :name "handle0"))
     (send link1 :assoc handle0)
     (setq handle1 (make-cascoords :pos #f(0 100 40) :rpy #f(-1.571 0.751 0) :name "handle1"))
     (send link1 :assoc handle1)
     
     ;; definition of :attention
     (setq attention0 (make-cascoords :pos #f(0 -62 70) :rpy #f(-1.571 0 0) :name "attention0"))
     (send link1 :assoc attention0)
     (setq attention1 (make-cascoords :pos #f(0 0 70) :rpy #f(-1.57 0 0) :name "attention1"))
     (send link1 :assoc attention1)
     
     ;; init-ending 
     (setq links (list link1))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)

     self))

  (:handle (&rest args) (forward-message-to-all (list  handle0 handle1) args))
  (:handle-handle0 (&rest args) (forward-message-to handle0 args))
  (:handle-handle1 (&rest args) (forward-message-to handle1 args))
  (:attention (&rest args) (forward-message-to-all (list  attention0 attention1) args))
  (:attention-attention0 (&rest args) (forward-message-to attention0 args))
  (:attention-attention1 (&rest args) (forward-message-to attention1 args))
  )

(defun measuring-cup-73b2 (&rest args) (instance* measuring-cup-73b2-object :init args))
