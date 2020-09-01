;; McDonald's M-size cup
(defclass mc-cup
    :super cascaded-link
    :slots ())
(defmethod mc-cup
    (:init
    (&rest args)
    (send-super* :init args)
    (let (mc-cup-body mc-cup-top mc-cup-straw)
        (setq mc-cup-body (make-solid-of-revolution '(#f(0 0 2) #f(30 0 2) #f(43 0 130) #f(45 0 130) #f(30 0 0) #f(0 0 0))))
        (send mc-cup-body :set-color #f(0.619 0 1))
        ;; top
        (setq mc-cup-top (make-torus '(#f(3 0 132) #f(45 0 132) #f(45 0 130) #f(3 0 130))))
        ;; straw
        (setq mc-cup-straw (make-torus '(#f(2.9 0 0) #f(2.9 0 200) #f(3.0 0 200) #f(3.0 0 0))))
        (send mc-cup-straw :set-color :yellow)
        ;; assembled
        ;; (send mc-cup-straw :transform (make-coords :pos #f(0 0 2) :rpy #f(0 0 0)))
        (send mc-cup-straw :translate (float-vector 0 0 2))
        (send mc-cup-body :assoc mc-cup-top)
        (send mc-cup-top :assoc mc-cup-straw)
        (setq cup (instance bodyset-link :init (make-cascoords) :bodies (list mc-cup-body mc-cup-top mc-cup-straw)))
        (setq links (list cup))
        (send self :assoc cup)
        
        (send self :init-ending)
        )
        self)
        )

;; McDonald's paper bag
;; In the future, I will add some methods that makes the bag folded, crumpled up.
(defclass mc-paper-bag-m
    :super cascaded-link
    :slots ())
(defmethod mc-paper-bag-m
    (:init
    (&rest args)
    (send-super* :init args)
    (let (mc-paper-bag-m-b1 mc-paper-bag-m-b2 pre-bag bag)
        (setq mc-paper-bag-m-b1 (make-cube 170 100 270))
        (setq mc-paper-bag-m-b2 (make-cube 168 98 270))
        (send mc-paper-bag-m-b2 :transform (make-coords :pos #f(0 0 2)))
        (setq pre-bag (body- mc-paper-bag-m-b1 mc-paper-bag-m-b2))
        (setq bag (instance bodyset-link :init (make-cascoords) :bodies (list pre-bag)))
        (send bag :transform (make-coords :pos #f(0 0 135) :rpy #f(0 0 0)))

        (send self :assoc bag)
        (setq links (list bag))

        (send self :init-ending)
        )
    self)
)    

(defclass mc-paper-bag-s
    :super cascaded-link
    :slots ())
(defmethod mc-paper-bag-s
    (:init
    (&rest args)
    (send-super* :init args)
    (let (mc-paper-bag-s-b1 mc-paper-bag-s-b2)
        (setq mc-paper-bag-s-b1 (make-cube 120 80 200))
        (setq mc-paper-bag-s-b2 (make-cube 118 78 200))
        (send mc-paper-bag-s-b2 :transform (make-coords :pos #f(0 0 2)))
        (setq pre-bag (body- mc-paper-bag-s-b1 mc-paper-bag-s-b2))
        (setq bag (instance bodyset-link :init (make-cascoords) :bodies (list pre-bag)))
        (send bag :transform (make-coords :pos #f(0 0 100) :rpy #f(0 0 0)))

        (send self :assoc bag)
        (setq links (list bag))
        (send self :init-ending)
        )
        self)
    )



;; asus-rog-rtx-2080ti-box
;; (setq asus-rog-rtx-2080ti-box)
