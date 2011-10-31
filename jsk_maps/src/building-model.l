;;
;; floor-model is a plane-type scene model for ROS navigation.
;; building-model is a set of map pieces.
;; each element of floors slot is a ROS map.
;;

(defun make-floor-plane (imgfile resolution cx cy &key (name))
  (let* ((map-img (read-image-file imgfile))
	 (width (* resolution (send map-img :width)))
	 (height (* resolution (send map-img :height)))
	 (map (make-cube width height 1)) top-face
	 nav-scene)
    (send map :translate-vertices
	  (float-vector (+ (/ height 2) cy) (- 0 (/ width 2) cx) 0))
    (send map :rotate-vertices pi/2 :z)
    (setq top-face (find-if
		    #'(lambda (f) (equal (send f :id) '(:top)))
		    (send map :faces)))
    ;;(setf (get map-img :texture-max-width) 2000)
    ;;(setf (get map-img :texture-max-height) 2000)
    (send map :paste-texture-to-face top-face :image map-img)
    (send map :name name)
    map ))

(defclass nav-scene-model
  :super scene-model
  :slots (mapobj))
(defmethod nav-scene-model
  (:map (&optional map) (if map (setq mapobj map) mapobj)))


(defclass building-model
  :super scene-model
  :slots ())
(defmethod building-model
  (:floors
   (&optional _flos)
   (if _flos
       (setq objs (mapcan #'(lambda(x)(send self :find-object x)) _flos))
     (remove-if-not #'(lambda(x)(derivedp x nav-scene-model)) objs)))
  (:floor<
   (name1 name2)
   (let ((p1 (find name1 objs :test #'equal :key #'(lambda(x)(send x :name))))
	 (p2 (find name2 objs :test #'equal :key #'(lambda(x)(send x :name)))))
     (< (elt (send p1 :worldpos) 2)
	(elt (send p2 :worldpos) 2))))
  (:current-floor
   (pos &optional (dist 1000))
   (if (derivedp pos coordinates) (setq pos (send pos :worldpos))) ;; for coordinates
   (let ((colbox (make-cube dist dist dist)))
     (send colbox :translate pos)
     (dolist (floor objs)
       (when (derivedp floor nav-scene-model)
	 (setq res (car (pqp-collision-distance colbox (send floor :map))))
	 (if (equal 0.0 res)
	     (return-from :current-floor (send floor :name)))
	 ))))
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