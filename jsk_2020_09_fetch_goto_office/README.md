```lisp
;; to elevator
(send *ri* :move-to (make-coords :pos #f(2072 -30746 0) :rpy (float-vector -pi/2 0 0)) :frame-id "/map")
;; enter elevator
(send *ri* :go-pos 3.5 0 180)
```