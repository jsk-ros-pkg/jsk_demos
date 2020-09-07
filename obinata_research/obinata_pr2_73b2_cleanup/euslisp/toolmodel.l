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
