(defclass plane-building-model
  :super scene-model
  :slots (floors rooms))
(defmethod plane-building-model
  (:floors ;; lower-floor to upper-floor ex.("1F" "2F" "3F" "4F")
   (&optional namelist)
   (if namelist
       (setq floors (mapcan #'(lambda(x)(send self :find-object x)) namelist))
     floors))
  (:floor<
   (name1 name2)
   (let ((names (mapcar #'(lambda(x)(send x :name)) floors)))
     (when (and (member name1 names :test #'equal) (member name2 names :test #'equal))
       (< (position name1 names :test #'equal) (position name2 names :test #'equal)))))
  (:rooms ;; this is sub-scene
   (&optional rms)
   (cond ((null rms) rooms)
		 ((listp rms) (setq rooms rms))
		 ((atomp rms) (push rms rooms))))
  (:current-floor
   (pos)
   (if (derivedp pos coordinates) (setq pos (send pos :worldpos))) ;; for coordinates
   (dolist (floor floors)
     (let* (re footpos1 footpos2)
       (setq re floor)
       (send re :worldpos)
       (setq footpos1
	     (send (find '(:top) (send re :faces)
			 :key #'(lambda(x)(send x :id)) :test #'equal)
		   :foot pos)
	     footpos2
	     (send (find '(:bottom) (send re :faces)
			 :key #'(lambda(x)(send x :id)) :test #'equal)
		   :foot pos))
       (if (not (eq :outside (send re :insidep (scale 0.5 (v+ footpos1 footpos2)))))
	   (return-from :current-floor (send floor :name)))
       )))
  )

;; arglist := cascoords or (arglist of make-cascoords)
;; children := (arglist of make-cascoords)
(defun make-cascoords-tree (arglist &rest children)
  (if children
	  (let (par chi)
		(if (derivedp arglist cascaded-coords)
			(setq par arglist)
		  (setq par (apply #'make-cascoords arglist)))
		(setq chi (mapcar #'make-cascoords-tree children))
		(mapcar #'(lambda(x)(send x :transform par :world)) chi)
		(mapcar #'(lambda(x)(send par :assoc x)) chi)
		par)
	(apply #'make-cascoords arglist))
  )