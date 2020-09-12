;; McDonald's M-size cup
(defclass mc-cup-object
    :super cascaded-link
    :slots (handle0))
(defmethod mc-cup-object
    (:init
    (&rest args)
    (send-super* :init args)
    (let (mc-cup-body mc-cup-top mc-cup-straw cup)
        (setq mc-cup-body (make-solid-of-revolution (list (float-vector 0 0 2) (float-vector 30 0 2) (float-vector 43 0 130) (float-vector 45 0 130) (float-vector 30 0 0) (float-vector 0 0 0))))
        (send mc-cup-body :set-color #f(0.619 0 1))
        ;; top
        (setq mc-cup-top (make-torus (list (float-vector 3 0 132) (float-vector 45 0 132) (float-vector 45 0 130) (float-vector 3 0 130))))
        ;; straw
        (setq mc-cup-straw (make-torus (list (float-vector 2.9 0 0) (float-vector 2.9 0 200) (float-vector 3.0 0 200) (float-vector 3.0 0 0))))
        (send mc-cup-straw :set-color :yellow)
        (send mc-cup-straw :translate (float-vector 0 0 2))
        (send mc-cup-body :assoc mc-cup-top)
        (send mc-cup-top :assoc mc-cup-straw)
        (setq cup (instance bodyset-link :init (make-cascoords) :bodies (list mc-cup-body mc-cup-top mc-cup-straw)))
        (setq links (list cup))
        (send self :assoc cup)
        
        (setq handle0 (make-cascoords :pos #f(0 0 60) :name "handle0"))
        (send cup :assoc handle0)

        (setq links (list cup))
        (setq joint-list (list))
        (send self :init-ending)
        (send-all links :worldcoords)

        self))

        (:handle (&rest args) (forward-message-to-all (list handle0) args))
    )

(defun mc-cup (&rest args) (instance* mc-cup-object :init args))


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
