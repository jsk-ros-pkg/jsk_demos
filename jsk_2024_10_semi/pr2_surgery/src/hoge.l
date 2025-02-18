(setq a nil)
(push 0 a)
(push 1 a)
(push 2 a)
(setq a (reverse a))
(print a)
(dolist (b a)
 (print b))

(print "--")
(setq a nil)
(setq a (append a (list 0)))
(setq a (append a (list 1)))
(setq a (append a (list 2)))
(print a)
(dolist (b a)
  (print b)) ;;






