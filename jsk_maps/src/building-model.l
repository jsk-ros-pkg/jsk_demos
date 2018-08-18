;;
;; floor-model is a plane-type scene model for ROS navigation.
;; building-model is a set of map pieces.
;; each element of floors slot is a ROS map.
;;

(defun make-floor-plane (imgfile resolution cx cy &key (name))
  (let* ((map-img (read-image-file imgfile))
	 (width (* resolution (send map-img :width)))
	 (height (* resolution (send map-img :height)))
	 (map (make-cube height width 1)) top-face
	 nav-scene)
    (send map :rotate-vertices pi/2 :z)
    (send map :translate-vertices
	  (float-vector (+ (/ width 2) cx) (+ (/ height 2) cy) 0))
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
       (setq objs (mapcan #'(lambda(x)(send self :find-object x)) _flos)))
   (setq _flos (remove-if-not #'(lambda(x)(derivedp x nav-scene-model)) objs))
   (sort _flos #'(lambda (x y) (<= (elt (send x :worldpos) 2)
                                   (elt (send y :worldpos) 2))))
   _flos)
  (:floor<
   (name1 name2)
   (let ((p1 (find name1 objs :test #'equal :key #'(lambda(x)(send x :name))))
	 (p2 (find name2 objs :test #'equal :key #'(lambda(x)(send x :name)))))
     (< (elt (send p1 :worldpos) 2)
	(elt (send p2 :worldpos) 2))))
  (:current-floor
   (pos)
   (cond
    ((derivedp pos cascaded-link) (setq pos (send pos :centroid)))
    ((derivedp pos coordinates) (setq pos (send pos :worldpos))))
   (let ((cur-floor (car (send self :floors))))
     (if (< (elt pos 2) (elt (send cur-floor :worldpos) 2))
         (return-from :current-floor nil)
       (dolist (nxt-floor (cdr (send self :floors)))
         (when (< (elt (send cur-floor :worldpos) 2)
                  (elt pos 2)
                  (elt (send nxt-floor :worldpos) 2))
           (return-from :current-floor (send cur-floor :name)))
         (setq cur-floor nxt-floor)))
     (send cur-floor :name)))
  )

;; rooms
(defmethod building-model
  (:rooms
   (&optional flostr)
   (unless flostr
     (return-from :rooms
       (flatten (mapcar #'(lambda (f) (send self :rooms (send f :name)))
                        (remove-if-not #'(lambda (x) (derivedp x nav-scene-model)) objs)))))
   (let ((scn (flatten (mapcar #'(lambda (x) (send x :objects)) (send self :find-object flostr)))))
     (remove-if-not #'(lambda (x) (derivedp x scene-model)) scn)))
  (:current-room
   (pos &optional (dist 3000))
   (when (derivedp pos coordinates) (setq pos (send pos :worldpos)))
   (let ((colbox (make-cylinder 250 dist)) bbox) ;; foot print width is 500mm
     (send colbox :translate pos)
     (dolist (room (send self :rooms))
       (setq bbox (bounding-box-union
                   (send-all (remove-if-not #'(lambda (b) (derivedp b faceset)) (send room :bodies)) :box)))
       (setq res (car (pqp-collision-distance colbox (convex-hull-3d (send (send bbox :body) :vertices)))))
       (when (equal 0.0 res)
         (return-from :current-room (send room :name)))))))

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
