;(send *prolog* :query
;  (list->prolog
;   `((findall #(:_P :_X) (bayes_probability_given ,omicsloc :_X ,rdftype :_P) :Ps))))

;; initial
;((0.239389 "http://ias.cs.tum.edu/kb/knowrob.owl#Kitchen") (0.130637 "http://ias.cs.tum.edu/kb/knowrob.owl#OfficeRoom") (0.063706 "http://ias.cs.tum.edu/kb/knowrob.owl#Hallway") (0.071175 "http://ias.cs.tum.edu/kb/knowrob.owl#Classroom") (0.07217 "http://ias.cs.tum.edu/kb/knowrob.owl#ConferenceRoom") (0.206406 "http://ias.cs.tum.edu/kb/knowrob.owl#RestaurantSpace") (0.071669 "http://ias.cs.tum.edu/kb/knowrob.owl#LockerRoom") (0.07217 "http://ias.cs.tum.edu/kb/knowrob.owl#AuditoriumRoom") (0.072678 "http://ias.cs.tum.edu/kb/knowrob.owl#Laboratory"))

;((("RoomT" "http://ias.cs.tum.edu/kb/knowrob.owl#Kitchen")) (("RoomT" "http://ias.cs.tum.edu/kb/knowrob.owl#OfficeRoom")) (("RoomT" "http://ias.cs.tum.edu/kb/knowrob.owl#RestaurantSpace")) (("RoomT" "http://ias.cs.tum.edu/kb/knowrob.owl#Kitchen")) (("RoomT" "http://ias.cs.tum.edu/kb/knowrob.owl#Kitchen")) (("RoomT" "http://ias.cs.tum.edu/kb/knowrob.owl#Kitchen")))
;; -> pareto
;;(lmeds (mapcar #'(lambda(x)(list (log (elt x 0)) (log (+ 0.5 (elt x 1))))) '((1 4)(2 1)(3 1)(4 0)(5 0)(6 0)(7 0)(8 0)(9 0))))
;;((0.0 1.50408) (0.693147 0.405465) (1.09861 0.405465) (1.38629 -0.693147) (1.60944 -0.693147) (1.79176 -0.693147) (1.94591 -0.693147) (2.07944 -0.693147))
;(1.26691 -1.0347)
;; ->  3.54987 * x^-1.0347

(defun lmeds (lst) ;; vec2
  (let ((n (length lst)) sx sy sxy sx2)
    (setq sx (apply #'+ (mapcar #'(lambda(v)(elt v 0)) lst))
	  sy (apply #'+ (mapcar #'(lambda(v)(elt v 1)) lst))
	  sxy (apply #'+ (mapcar #'(lambda(v)(* (elt v 0)(elt v 1))) lst))
	  sx2 (apply #'+ (mapcar #'(lambda(v)(* (elt v 0)(elt v 0))) lst)))
    (list (/ (- (* sx sxy) (* sx2 sy)) (- (* sx sx) (* n sx2)))
	  (/ (- (* sx sy) (* n sxy)) (- (* sx sx) (* n sx2))))
    ))

(defun pareto* (count alpha)
  (let ((n (length count)) var)
    (dotimes (i n)
      (push (exp (* (log (1+ i)) alpha)) var))
    (nreverse var)
    ))
