(setq c1 (make-cube 580 350 10))
(setq c2 (make-cube 580 350 10))
(setq c3 (make-cube 10 350 900))
(setq c4 (make-cube 10 350 900))
(setq c5 (make-cube 580 10 900))
(setq cube (make-cube 580 350 900))

(send c1 :locate #f(0 0 900))
(send c3 :locate #f(-290 0 450))
(send c4 :locate #f(290 0 450))
(send c5 :locate #f(0 175 450))
(send cube :locate #f(0 0 0))

(send c2 :assoc c1)
(send c2 :assoc c3)
(send c2 :assoc c4)
(send c2 :assoc c5)
(send cube :locate #f(0 0 450))

(send c2 :locate #f(0 0 900))

(setq shelf (instance bodyset :init (make-cascoords) :bodies (list c1 c2 c3 c4 c5 cube)))