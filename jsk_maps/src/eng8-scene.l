(load "models/elevator_call_panel-object.l")
(load "models/elevator_inside_panel-object.l")

(load "package://jsk_maps/src/building-model.l")

(defun make-eng8-scene ()
  (let* (scene (map (make-eng8-map-map)) floor-names spots)
	(setq floor-names
	      (mapcar #'(lambda(x)(send x :name)) (send map :descendants)))

	;; B1F
	(make-cascoords-tree
	 (find "/eng8/b1f" (send map :descendants)
	       :test #'string= :key #'(lambda(x)(send x :name)))
	 '(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
			:pos #f(3200.0 -29200.0 0.0)
			:name "/eng8/b1f/elevator_call_panel-front")
	 '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
			:pos #f(4100.0 -31300.0 0.0)
			:name "/eng8/b1f/elevator_inside_panel-front")
	 '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
			:pos #f(4000.0 -29000.0 0.0)
			:name "/eng8/b1f/elevator-outside")
	 '(:pos #f(3100 -29900 920) :rpy #f(1.5706 0 0)
			:name "/eng8/b1f/elevator_call_panel")
	 '(:pos #f(4850 -31550 1000) :rpy #f(3.1416 0 0)
			:name "/eng8/b1f/elevator_inside_panel"))
	;; 1F
	(make-cascoords-tree
	 (find "/eng8/1f" (send map :descendants)
	       :test #'string= :key #'(lambda(x)(send x :name)))
	 '(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
			:pos #f(3200.0 -29100.0 0.0)
			:name "/eng8/1f/elevator_call_panel-front")
	 '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
			:pos #f(4100.0 -31300.0 0.0)
			:name "/eng8/1f/elevator_inside_panel-front")
	 '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
			:pos #f(4000.0 -29000.0 0.0)
			:name "/eng8/1f/elevator-outside")
	 '(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
			:pos #f(15500.0 -33000.0 0.0)
			:name "/eng8/1f/pamphlet-front")
	 '(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
			:pos #f(-1170.0 -27600.0 0.0)
			:name "/eng8/1f/student-affairs")
	 '(:pos #f(3100 -29900 920) :rpy #f(1.5706 0 0)
			:name "/eng8/1f/elevator_call_panel")
	 '(:pos #f(4850 -31550 1000) :rpy #f(3.1416 0 0)
			:name "/eng8/1f/elevator_inside_panel"))
	;; 6F
	(make-cascoords-tree
	 (find "/eng8/6f" (send map :descendants)
	       :test #'string= :key #'(lambda(x)(send x :name)))
	 '(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
			:pos #f(7500.0 2300.0 0.0)
			:name "/eng8/6f/room602-kitchen")
	 '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
			:pos #f(7600.0 2700.0 0.0)
			:name "/eng8/6f/room602-microwave")
	 '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
			:pos #f(7600.0 1100.0 0.0)
			:name "/eng8/6f/room602-refrigerator")
	 '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
			:pos #f(4000.0 4300.0 0.0)
			:name "/eng8/6f/room602-shelf")
	 '(:rot #2f((-0.866025 -0.5 0.0) (0.5 -0.866025 0.0) (0.0 0.0 1.0))
			:pos #f(1300.0 3800.0 0.0)
			:name "/eng8/6f/room602-washingmachine")
	 '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
			:pos #f(1750.0 850.0 0.0)
			:name "/eng8/6f/room602-base")
	 '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
			:pos #f(-400.0 100.0 0.0)
			:name "/eng8/6f/room602-front")
	 '(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
			:pos #f(3200.0 -29100.0 0.0)
			:name "/eng8/6f/elevator_call_panel-front")
	 '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
			:pos #f(4100.0 -31300.0 0.0)
			:name "/eng8/6f/elevator_inside_panel-front")
	 '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
			:pos #f(4000.0 -29000.0 0.0)
			:name "/eng8/6f/elevator-outside")
	 '(:pos #f(3100 -29900 920) :rpy #f(1.5706 0 0)
			:name "/eng8/6f/elevator_call_panel")
	 '(:pos #f(4850 -31550 1000) :rpy #f(3.1416 0 0)
			:name "/eng8/6f/elevator_inside_panel"))

	(setq scene
		  (instance
		   plane-building-model :init :name "/eng8"
		   :objects
		   (append
			(list
			 map
			 (elevator_call_panel :name "elevator_call_panel")
			 (elevator_inside_panel :name "elevator_inside_panel"))
			(flatten (send map :inheritance)))
		   ))
    ;; scene -> map
    (send scene :assoc map)
    (send scene :floors floor-names) ;; set floor information

	;; visualize region
	(mapcar #'(lambda(x)(let ((obj (car (send scene :find-object (concatenate string x "/region")))))
						  (send obj :set-color #f(0 1 0 0.3))))
			floor-names)

    (dolist (obj (send scene :objects))
      ;; first charactor of obj-name must be slash
      (when (not (eq (char (send obj :name) 0) #\/))
		(send obj :name (format nil "/~A" (send obj :name)))))

    scene)

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
