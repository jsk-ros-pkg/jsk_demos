(load "package://jsk_maps/euslisp/eng8-map.l")
(load "models/elevator_call_panel-object.l")
(load "models/elevator_inside_panel-object.l")

(defclass plane-building-model
  :super scene-model
  :slots (floors))
(defmethod plane-building-model
  (:floors ;; lower-floor to upper-floor ex.("1F" "2F" "3F" "4F")
   (&optional namelist)
   (if namelist
       (setq floors (mapcar #'(lambda(x)(send self :spot x)) namelist))
     floors))
  (:floor<
   (name1 name2)
   (let ((names (mapcar #'(lambda(x)(send x :name)) floors)))
     (when (and (member name1 names :test #'equal) (member name2 names :test #'equal))
       (< (position name1 names :test #'equal) (position name2 names :test #'equal)))))
  (:current-floor
   (pos)
   (if (derivedp pos coordinates) (setq pos (send pos :worldpos))) ;; for coordinates
   (dolist (floor floors)
     (let* (re footpos1 footpos2)
       (setq re (find (format nil "~A/region" (send floor :name))
		      (flatten (send floor :inheritance))
		      :key #'(lambda(x)(send x :name)) :test #'equal))
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

(defun make-eng8-scene ()
  (let* ((map (make-eng8-map-map))
	 (scene
	  (instance
	   plane-building-model :init :name "/eng8"
	   :objects
	   (append
	    (list
	     map
	     (elevator_call_panel)
	     (elevator_inside_panel)
	    ;; B1F
	    (make-cascoords :rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
			    :pos #f(18200.0 -29200.0 0.0)
			    :name "/eng8/b1f/elevator_call_panel-front")
	    (make-cascoords :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
			    :pos #f(19100.0 -31300.0 0.0)
			    :name "/eng8/b1f/elevator_inside_panel-front")
	    (make-cascoords :rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
			    :pos #f(19000.0 -29000.0 0.0)
			    :name "/eng8/b1f/elevator-outside")
	    ;; 1F
	    (make-cascoords :rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
			    :pos #f(-26800.0 -29100.0 0.0)
			    :name "/eng8/1f/elevator_call_panel-front")
	    (make-cascoords :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
			    :pos #f(-25900.0 -31300.0 0.0)
			    :name "/eng8/1f/elevator_inside_panel-front")
	    (make-cascoords :rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
			    :pos #f(-26000.0 -29000.0 0.0)
			    :name "/eng8/1f/elevator-outside")
	    (make-cascoords :rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
			    :pos #f(-14500.0 -33000.0 0.0)
			    :name "/eng8/1f/pamphlet-front")
	    (make-cascoords :rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
			    :pos #f(-31170.0 -27600.0 0.0)
			    :name "/eng8/1f/student-affairs")
	    ;; 6F
	    (make-cascoords :rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
			    :pos #f(7500.0 2300.0 0.0)
			    :name "/eng8/6f/room602-kitchen")
	    (make-cascoords :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
			    :pos #f(7600.0 2700.0 0.0)
			    :name "/eng8/6f/room602-microwave")
	    (make-cascoords :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
			    :pos #f(7600.0 1100.0 0.0)
			    :name "/eng8/6f/room602-refrigerator")
	    (make-cascoords :rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
			    :pos #f(4000.0 4300.0 0.0)
			    :name "/eng8/6f/room602-shelf")
	    (make-cascoords :rot #2f((-0.866025 -0.5 0.0) (0.5 -0.866025 0.0) (0.0 0.0 1.0))
			    :pos #f(1300.0 3800.0 0.0)
			    :name "/eng8/6f/room602-washingmachine")
	    (make-cascoords :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
			    :pos #f(1750.0 850.0 0.0)
			    :name "/eng8/6f/room602-base")
	    (make-cascoords :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
			    :pos #f(-400.0 100.0 0.0)
			    :name "/eng8/6f/room602-front")
	    (make-cascoords :rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
			    :pos #f(3200.0 -29100.0 0.0)
			    :name "/eng8/6f/elevator_call_panel-front")
	    (make-cascoords :rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
			    :pos #f(4100.0 -31300.0 0.0)
			    :name "/eng8/6f/elevator_inside_panel-front")
	    (make-cascoords :rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
			    :pos #f(4000.0 -29000.0 0.0)
			    :name "/eng8/6f/elevator-outside")
	    ;; object position
	    (make-cascoords :pos #f(18100 -29900 920) :rpy #f(1.5706 0 0)
			    :name "/eng8/b1f/elevator_call_panel")
	    (make-cascoords :pos #f(19850 -31550 1000) :rpy #f(3.1416 0 0)
			    :name "/eng8/b1f/elevator_inside_panel")
	    (make-cascoords :pos #f(-26900 -29900 920) :rpy #f(1.5706 0 0)
			    :name "/eng8/1f/elevator_call_panel")
	    (make-cascoords :pos #f(-25150 -31550 1000) :rpy #f(3.1416 0 0)
			    :name "/eng8/1f/elevator_inside_panel")
	    (make-cascoords :pos #f(3100 -29900 920) :rpy #f(1.5706 0 0)
			    :name "/eng8/6f/elevator_call_panel")
	    (make-cascoords :pos #f(4850 -31550 1000) :rpy #f(3.1416 0 0)
			    :name "/eng8/6f/elevator_inside_panel"))
	    (flatten (send map :descendants))
	    ))))
    ;; scene -> map
    (send scene :assoc map)
    (send scene :floors '("/eng8/b1f" "/eng8/1f" "/eng8/6f")) ;; set floor information

    (dolist (obj (send scene :objects))
      ;; map -> unassoc obj
      (unless (find obj (flatten (send scene :inheritance)))
	(send map :assoc obj))
      ;; first charactor of obj-name must be slash
      (when (not (eq (char (send obj :name) 0) #\/))
	(send obj :name (format nil "/~A" (send obj :name)))))
    scene))

;; example
;; (load "package://pr2eus/pr2-interface")
;; (ros::roseus "eng8_navigation")
;; (defvar *tfl* (instance ros::transform-listener :init))
;; (setq *ri* (instance pr2-interface :init))
;; (objects (list (setq *eng8-scene*  (make-eng8-scene))))
;; (move-to (car (send *eng8-scene* :find-object "/eng8/6f/room602-front")))
;;
;; debug
#|
(defun move-to-test (&optional (spot "/eng8/6f/room602-base"))
  (load "eng8-map-scene.l")
  (objects (list (setq *eng8-scene*  (make-eng8-scene))))
  (send *irtviewer* :look-all (make-cube 1000 1000 10000))
  (send (car (send *eng8-scene* :find-object spot)) :draw-on :flush t :size 1000 :color #f(1 0 0))
  (move-to (car (send *eng8-scene* :find-object spot)))
  )
|#
