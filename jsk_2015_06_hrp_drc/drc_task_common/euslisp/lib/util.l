(defun c* (a b)
  (send a :transform b)
  )

(defun z0 (pos)
  (float-vector (elt pos 0) (elt pos 1) 0))
(defun zset (pos z)
  (float-vector (elt pos 0) (elt pos 1) z))

