;; bottle
(defclass bottle-2l
    :super cascaded-link
    :slots ())
(defmethod bottle-2l
    (:init
    (&rest args)
    (send-super* :init args)
    (let (bottle-b bottle)
        (setq bottle-b (make-solid-of-revolution (list (float-vector 0 0 300) (float-vector 14 0 300) (float-vector 14 0 280) (float-vector 44 0 230) (float-vector 44 0 0) (float-vector 0 0 0)))) 
        (send bottle-b :set-color :white)
        (setq bottle (instance bodyset-link :init (make-cascoords) :bodies (list bottle-b)))

        (send self :assoc bottle)
        (setq links (list bottle))
        
        (send self :init-ending)
        )
    self)
)

;; seasonings
(defclass ketchup
    :super cascaded-link
    :slots ())
(defmethod ketchup
    (:init
    (&rest args)
    (send-super* :init args)
    (let (bottle-b bottle)
        (setq bottle-b (make-solid-of-revolution (list (float-vector 0 0 190) (float-vector 15 0 190) (float-vector 15 0 160) (float-vector 35 0 90) (float-vector 35 0 30) (float-vector 30 0 0) (float-vector 0 0 0))))
        (send bottle-b :set-color :red)
        (setq bottle (instance bodyset-link :init (make-cascoords) :bodies (list bottle-b)))

        (send self :assoc bottle)
        (setq links (list bottle))
        
        (send self :init-ending)
        )
    self)
)

(defclass sauce
    :super cascaded-link
    :slots ())
(defmethod sauce
    (:init
    (&rest args)
    (send-super* :init args)
    (let (bottle-b bottle)
        (setq bottle-b (make-solid-of-revolution (list (float-vector 0 0 190) (float-vector 15 0 190) (float-vector 15 0 160) (float-vector 35 0 90) (float-vector 35 0 30) (float-vector 30 0 0) (float-vector 0 0 0))))
        (send bottle-b :set-color :brown)
        (setq bottle (instance bodyset-link :init (make-cascoords) :bodies (list bottle-b)))

        (send self :assoc bottle)
        (setq links (list bottle))
        
        (send self :init-ending)
        )
    self)
)

(defclass trashbox
  :super cascaded-link
  :slots (sensors
         handle0 handle1
         ))
(defmethod trashbox  (:init
   (&rest args &key (name "trashbox") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (c bc
         blink0
                 )
     (send-super* :init :name name args)

     ;; definition of link

     ;; definition of :room73b2-trashbox0-bodyset2
     (setq bc (list
       (instance faceset :init :faces (list
         (instance face :init :vertices (list (float-vector 200.0 100.0 10.0) (float-vector 150.0 150.0 10.0) (float-vector -150.0 150.0 10.0) (float-vector -200.0 100.0 10.0) (float-vector -200.0 -100.0 10.0) (float-vector -150.0 -150.0 10.0) (float-vector 150.0 -150.0 10.0) (float-vector 200.0 -100.0 10.0)))
         (instance face :init :vertices (list (float-vector 150.0 150.0 0.0) (float-vector 200.0 100.0 0.0) (float-vector 200.0 -100.0 0.0) (float-vector 150.0 -150.0 0.0) (float-vector -150.0 -150.0 0.0) (float-vector -200.0 -100.0 0.0) (float-vector -200.0 100.0 0.0) (float-vector -150.0 150.0 0.0)))
         (instance face :init :vertices (list (float-vector 200.0 100.0 10.0) (float-vector 200.0 100.0 0.0) (float-vector 150.0 150.0 0.0) (float-vector 150.0 150.0 10.0)))
         (instance face :init :vertices (list (float-vector 200.0 -100.0 10.0) (float-vector 200.0 -100.0 0.0) (float-vector 200.0 100.0 0.0) (float-vector 200.0 100.0 10.0)))
         (instance face :init :vertices (list (float-vector 150.0 -150.0 10.0) (float-vector 150.0 -150.0 0.0) (float-vector 200.0 -100.0 0.0) (float-vector 200.0 -100.0 10.0)))
         (instance face :init :vertices (list (float-vector -150.0 -150.0 10.0) (float-vector -150.0 -150.0 0.0) (float-vector 150.0 -150.0 0.0) (float-vector 150.0 -150.0 10.0)))
         (instance face :init :vertices (list (float-vector -200.0 -100.0 10.0) (float-vector -200.0 -100.0 0.0) (float-vector -150.0 -150.0 0.0) (float-vector -150.0 -150.0 10.0)))
         (instance face :init :vertices (list (float-vector -200.0 100.0 10.0) (float-vector -200.0 100.0 0.0) (float-vector -200.0 -100.0 0.0) (float-vector -200.0 -100.0 10.0)))
         (instance face :init :vertices (list (float-vector -150.0 150.0 10.0) (float-vector -150.0 150.0 0.0) (float-vector -200.0 100.0 0.0) (float-vector -200.0 100.0 10.0)))
         (instance face :init :vertices (list (float-vector 150.0 150.0 10.0) (float-vector 150.0 150.0 0.0) (float-vector -150.0 150.0 0.0) (float-vector -150.0 150.0 10.0)))
       ))
       (instance faceset :init :faces (list
         (instance face :init :vertices (list (float-vector 200.0 100.0 500.0) (float-vector 189.99 99.99 500.0) (float-vector 190.0 -100.0 500.0) (float-vector 150.0 -140.0 500.0) (float-vector -150.0 -140.0 500.0) (float-vector -190.0 -100.0 500.0) (float-vector -190.0 100.0 500.0) (float-vector -150.0 140.0 500.0) (float-vector 150.0 140.0 500.0) (float-vector 190.0 100.0 500.0) (float-vector 199.99 100.01 500.0) (float-vector 150.0 150.0 500.0) (float-vector -150.0 150.0 500.0) (float-vector -200.0 100.0 500.0) (float-vector -200.0 -100.0 500.0) (float-vector -150.0 -150.0 500.0) (float-vector 150.0 -150.0 500.0) (float-vector 200.0 -100.0 500.0)))
         (instance face :init :vertices (list (float-vector 189.99 99.99 0.0) (float-vector 200.0 100.0 0.0) (float-vector 200.0 -100.0 0.0) (float-vector 150.0 -150.0 0.0) (float-vector -150.0 -150.0 0.0) (float-vector -200.0 -100.0 0.0) (float-vector -200.0 100.0 0.0) (float-vector -150.0 150.0 0.0) (float-vector 150.0 150.0 0.0) (float-vector 199.99 100.01 0.0) (float-vector 190.0 100.0 0.0) (float-vector 150.0 140.0 0.0) (float-vector -150.0 140.0 0.0) (float-vector -190.0 100.0 0.0) (float-vector -190.0 -100.0 0.0) (float-vector -150.0 -140.0 0.0) (float-vector 150.0 -140.0 0.0) (float-vector 190.0 -100.0 0.0)))
         (instance face :init :vertices (list (float-vector 200.0 100.0 500.0) (float-vector 200.0 100.0 0.0) (float-vector 189.99 99.99 0.0) (float-vector 189.99 99.99 500.0)))
         (instance face :init :vertices (list (float-vector 200.0 -100.0 500.0) (float-vector 200.0 -100.0 0.0) (float-vector 200.0 100.0 0.0) (float-vector 200.0 100.0 500.0)))
         (instance face :init :vertices (list (float-vector 150.0 -150.0 500.0) (float-vector 150.0 -150.0 0.0) (float-vector 200.0 -100.0 0.0) (float-vector 200.0 -100.0 500.0)))
         (instance face :init :vertices (list (float-vector -150.0 -150.0 500.0) (float-vector -150.0 -150.0 0.0) (float-vector 150.0 -150.0 0.0) (float-vector 150.0 -150.0 500.0)))
         (instance face :init :vertices (list (float-vector -200.0 -100.0 500.0) (float-vector -200.0 -100.0 0.0) (float-vector -150.0 -150.0 0.0) (float-vector -150.0 -150.0 500.0)))
         (instance face :init :vertices (list (float-vector -200.0 100.0 500.0) (float-vector -200.0 100.0 0.0) (float-vector -200.0 -100.0 0.0) (float-vector -200.0 -100.0 500.0)))
         (instance face :init :vertices (list (float-vector -150.0 150.0 500.0) (float-vector -150.0 150.0 0.0) (float-vector -200.0 100.0 0.0) (float-vector -200.0 100.0 500.0)))
         (instance face :init :vertices (list (float-vector 150.0 150.0 500.0) (float-vector 150.0 150.0 0.0) (float-vector -150.0 150.0 0.0) (float-vector -150.0 150.0 500.0)))
         (instance face :init :vertices (list (float-vector 199.99 100.01 500.0) (float-vector 199.99 100.01 0.0) (float-vector 150.0 150.0 0.0) (float-vector 150.0 150.0 500.0)))
         (instance face :init :vertices (list (float-vector 190.0 100.0 500.0) (float-vector 190.0 100.0 0.0) (float-vector 199.99 100.01 0.0) (float-vector 199.99 100.01 500.0)))
         (instance face :init :vertices (list (float-vector 150.0 140.0 500.0) (float-vector 150.0 140.0 0.0) (float-vector 190.0 100.0 0.0) (float-vector 190.0 100.0 500.0)))
         (instance face :init :vertices (list (float-vector -150.0 140.0 500.0) (float-vector -150.0 140.0 0.0) (float-vector 150.0 140.0 0.0) (float-vector 150.0 140.0 500.0)))
         (instance face :init :vertices (list (float-vector -190.0 100.0 500.0) (float-vector -190.0 100.0 0.0) (float-vector -150.0 140.0 0.0) (float-vector -150.0 140.0 500.0)))
         (instance face :init :vertices (list (float-vector -190.0 -100.0 500.0) (float-vector -190.0 -100.0 0.0) (float-vector -190.0 100.0 0.0) (float-vector -190.0 100.0 500.0)))
         (instance face :init :vertices (list (float-vector -150.0 -140.0 500.0) (float-vector -150.0 -140.0 0.0) (float-vector -190.0 -100.0 0.0) (float-vector -190.0 -100.0 500.0)))
         (instance face :init :vertices (list (float-vector 150.0 -140.0 500.0) (float-vector 150.0 -140.0 0.0) (float-vector -150.0 -140.0 0.0) (float-vector -150.0 -140.0 500.0)))
         (instance face :init :vertices (list (float-vector 190.0 -100.0 500.0) (float-vector 190.0 -100.0 0.0) (float-vector 150.0 -140.0 0.0) (float-vector 150.0 -140.0 500.0)))
         (instance face :init :vertices (list (float-vector 189.99 99.99 500.0) (float-vector 189.99 99.99 0.0) (float-vector 190.0 -100.0 0.0) (float-vector 190.0 -100.0 500.0)))
       ))
       (instance faceset :init :faces (list
         (instance face :init :vertices (list (float-vector 210.0 100.0 510.0) (float-vector 189.99 99.99 510.0) (float-vector 190.0 -100.0 510.0) (float-vector 150.0 -140.0 510.0) (float-vector -150.0 -140.0 510.0) (float-vector -190.0 -100.0 510.0) (float-vector -190.0 100.0 510.0) (float-vector -150.0 140.0 510.0) (float-vector 150.0 140.0 510.0) (float-vector 190.0 100.0 510.0) (float-vector 209.99 100.01 510.0) (float-vector 150.0 160.0 510.0) (float-vector -150.0 160.0 510.0) (float-vector -210.0 100.0 510.0) (float-vector -210.0 -100.0 510.0) (float-vector -150.0 -160.0 510.0) (float-vector 150.0 -160.0 510.0) (float-vector 210.0 -100.0 510.0)))
         (instance face :init :vertices (list (float-vector 189.99 99.99 500.0) (float-vector 210.0 100.0 500.0) (float-vector 210.0 -100.0 500.0) (float-vector 150.0 -160.0 500.0) (float-vector -150.0 -160.0 500.0) (float-vector -210.0 -100.0 500.0) (float-vector -210.0 100.0 500.0) (float-vector -150.0 160.0 500.0) (float-vector 150.0 160.0 500.0) (float-vector 209.99 100.01 500.0) (float-vector 190.0 100.0 500.0) (float-vector 150.0 140.0 500.0) (float-vector -150.0 140.0 500.0) (float-vector -190.0 100.0 500.0) (float-vector -190.0 -100.0 500.0) (float-vector -150.0 -140.0 500.0) (float-vector 150.0 -140.0 500.0) (float-vector 190.0 -100.0 500.0)))
         (instance face :init :vertices (list (float-vector 210.0 100.0 510.0) (float-vector 210.0 100.0 500.0) (float-vector 189.99 99.99 500.0) (float-vector 189.99 99.99 510.0)))
         (instance face :init :vertices (list (float-vector 210.0 -100.0 510.0) (float-vector 210.0 -100.0 500.0) (float-vector 210.0 100.0 500.0) (float-vector 210.0 100.0 510.0)))
         (instance face :init :vertices (list (float-vector 150.0 -160.0 510.0) (float-vector 150.0 -160.0 500.0) (float-vector 210.0 -100.0 500.0) (float-vector 210.0 -100.0 510.0)))
         (instance face :init :vertices (list (float-vector -150.0 -160.0 510.0) (float-vector -150.0 -160.0 500.0) (float-vector 150.0 -160.0 500.0) (float-vector 150.0 -160.0 510.0)))
         (instance face :init :vertices (list (float-vector -210.0 -100.0 510.0) (float-vector -210.0 -100.0 500.0) (float-vector -150.0 -160.0 500.0) (float-vector -150.0 -160.0 510.0)))
         (instance face :init :vertices (list (float-vector -210.0 100.0 510.0) (float-vector -210.0 100.0 500.0) (float-vector -210.0 -100.0 500.0) (float-vector -210.0 -100.0 510.0)))
         (instance face :init :vertices (list (float-vector -150.0 160.0 510.0) (float-vector -150.0 160.0 500.0) (float-vector -210.0 100.0 500.0) (float-vector -210.0 100.0 510.0)))
         (instance face :init :vertices (list (float-vector 150.0 160.0 510.0) (float-vector 150.0 160.0 500.0) (float-vector -150.0 160.0 500.0) (float-vector -150.0 160.0 510.0)))
         (instance face :init :vertices (list (float-vector 209.99 100.01 510.0) (float-vector 209.99 100.01 500.0) (float-vector 150.0 160.0 500.0) (float-vector 150.0 160.0 510.0)))
         (instance face :init :vertices (list (float-vector 190.0 100.0 510.0) (float-vector 190.0 100.0 500.0) (float-vector 209.99 100.01 500.0) (float-vector 209.99 100.01 510.0)))
         (instance face :init :vertices (list (float-vector 150.0 140.0 510.0) (float-vector 150.0 140.0 500.0) (float-vector 190.0 100.0 500.0) (float-vector 190.0 100.0 510.0)))
         (instance face :init :vertices (list (float-vector -150.0 140.0 510.0) (float-vector -150.0 140.0 500.0) (float-vector 150.0 140.0 500.0) (float-vector 150.0 140.0 510.0)))
         (instance face :init :vertices (list (float-vector -190.0 100.0 510.0) (float-vector -190.0 100.0 500.0) (float-vector -150.0 140.0 500.0) (float-vector -150.0 140.0 510.0)))
         (instance face :init :vertices (list (float-vector -190.0 -100.0 510.0) (float-vector -190.0 -100.0 500.0) (float-vector -190.0 100.0 500.0) (float-vector -190.0 100.0 510.0)))
         (instance face :init :vertices (list (float-vector -150.0 -140.0 510.0) (float-vector -150.0 -140.0 500.0) (float-vector -190.0 -100.0 500.0) (float-vector -190.0 -100.0 510.0)))
         (instance face :init :vertices (list (float-vector 150.0 -140.0 510.0) (float-vector 150.0 -140.0 500.0) (float-vector -150.0 -140.0 500.0) (float-vector -150.0 -140.0 510.0)))
         (instance face :init :vertices (list (float-vector 190.0 -100.0 510.0) (float-vector 190.0 -100.0 500.0) (float-vector 150.0 -140.0 500.0) (float-vector 150.0 -140.0 510.0)))
         (instance face :init :vertices (list (float-vector 189.99 99.99 510.0) (float-vector 189.99 99.99 500.0) (float-vector 190.0 -100.0 500.0) (float-vector 190.0 -100.0 510.0)))
       ))
       ))
     (dolist (b (cdr bc)) (send (car bc) :assoc b))
     (send (elt bc 0) :set-color :blue)
     (send (elt bc 1) :set-color :blue)
     (send (elt bc 2) :set-color :blue)
     (setq blink0 (instance bodyset-link :init (make-cascoords) :bodies bc :name :room73b2-trashbox0-bodyset2 :weight 1 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))

     ;; definition of assoc
     (send self :assoc blink0)

     ;; definition of end-coords

     ;; definition of joint


     ;; definition of :handle
     (setq handle0 (make-cascoords :pos (float-vector 200.0 0.0 500.0) :rpy (float-vector 0 -1.571 1.571) :name "handle0"))
     (send blink0 :assoc handle0)
     (setq handle1 (make-cascoords :pos (float-vector -200.0 0.0 500.0) :rpy (float-vector 0 -1.571 -1.571) :name "handle1"))
     (send blink0 :assoc handle1)
    
     ;; init-ending 
     (setq links (list blink0))
     (setq joint-list (list))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)

     self))

  (:handle (&rest args) (forward-message-to-all (list  handle0 handle1) args))
  (:handle-handle0 (&rest args) (forward-message-to handle0 args))
  (:handle-handle1 (&rest args) (forward-message-to handle1 args))
  (:attention (&rest args) (forward-message-to-all (list ) args))
  (:button (&rest args) (forward-message-to-all (list ) args))
  )

(defun trashbox (&rest args) (instance* trashbox :init args))

;; book
(defclass book
    :super cascaded-link
    :slots ())

;; magazine, stacked
(defclass magazine
    :super cascaded-link
    :slots ())

;; obinata tools
(setq eyeglasses-case (make-cube 170 60 40))
(send eyeglasses-case :set-color :black)
(setq obinata-wallet (make-cube 90 110 20))
(send obinata-wallet :set-color :gray)
(setq obinata-purse (make-cube 75 95 20))
(send obinata-purse :set-color :gray) 
