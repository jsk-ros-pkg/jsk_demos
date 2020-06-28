;; McDonald's M-size cup
;; body
(setq mc-cup-body (make-solid-of-revolution '(#f(0 0 2) #f(30 0 2) #f(43 0 130) #f(45 0 130) #f(30 0 0) #f(0 0 0))))
(send mc-cup-body :set-color #f(0.619 0 1))
;; top
(setq mc-cup-top (make-torus '(#f(3 0 132) #f(45 0 132) #f(45 0 130) #f(3 0 130))))
;; straw
(setq mc-cup-straw (make-torus '(#f(2.9 0 0) #f(2.9 0 200) #f(3.0 0 200) #f(3.0 0 0))))
(send mc-cup-straw :set-color :yellow)
;; assembled
(send mc-cup-straw :transform (make-coords :pos #f(0 0 2) :rpy #f(0 0 0)))
(send mc-cup-body :assoc mc-cup-top)
(send mc-cup-top :assoc mc-cup-straw)
(setq mc-cup (instance bodyset :init (make-cascoords) :bodies (list mc-cup-body mc-cup-top mc-cup-straw)))

;; McDonald's paper bag
;; In the future, I will add some methods that makes the bag folded, crumpled up.
(setq mc-paper-bag-m-b1 (make-cube 170 100 270))
(setq mc-paper-bag-m-b2 (make-cube 169 99 269.5))
(send mc-paper-bag-m-b2 :transform (make-coords :pos #f(0 0 0.5)))
;; (setq mc-paper-bag-m (body- mc-paper-bag-m-b1 mc-paper-bag-m-b2))

(setq mc-paper-bag-s)


;; McDonald's nugget box
