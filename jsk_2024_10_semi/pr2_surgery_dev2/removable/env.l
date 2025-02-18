(defun set-env()
  ;;cet center
  (setq *center* (make-cube 10 10 10))
  (send *center* :translate (float-vector *centerx* *centery* *centerz*))
  (send *center* :set-color :black)
  (setq *o* (make-cascoords :pos (send *center* :pos)))

  ;;Set desk
  (setq *desk* (make-cube *deskw* *deskw* *deskh*))
  (send *desk* :translate (float-vector *centerx* *centery* (/ *centerz* 2))) 
  (send *desk* :set-color :brown)

  ;;Set needle
  (setq *needle* (make-cylinder 0.5 70))
  (send *needle* :translate (v+ (float-vector 0 20 300) (send *center* :pos)))
  (send *needle* :set-color :yellow)
  (send *needle* :rotate #d90 :y)

  ;;Set hampen
  (setq *hampen* (make-cube 50 50 50))
  (send *hampen* :translate (v+ (float-vector 0 0 10) (send *center* :pos)))
  (send *hampen* :set-color :white)
  (send *hampen* :rotate #d45 :x)

  (send *desk* :put :left-coords
	(make-cascoords
       :coords (send (send *desk* :copy-worldcoords) :translate (float-vector 0 0 500))
       :parent *pr2*))
  (send *needle* :put :left-coords
	(make-cascoords
       :coords (send (send *needle* :copy-worldcoords) :translate (float-vector 0 0 -5))
       :parent *needle*)) 
)
