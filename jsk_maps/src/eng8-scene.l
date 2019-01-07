(load "package://jsk_maps/src/building-model.l")

(load "models/elevator_call_panel-object.l")
(load "models/elevator_inside_panel-object.l")

(load "models/room602-scene.l")
(load "models/room610-scene.l")

(defun make-eng8-b1f-scene ()
  (let* (scene map floor-names rooms spots objects)
    (setq map (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng8-b1f-0.05.pgm") 50 -7500 -75000 :name "/eng8/b1f/floor-plane"))

    ;; B1F
    (make-cascoords-tree
     map
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

    (setq scene
	  (instance nav-scene-model :init :name "/eng8/b1f"
		    :objects (append (list map) (send map :descendants))))
    (send scene :map map)
    scene ))


(defun make-eng8-1f-scene ()
  (let* (scene map floor-names rooms spots objects)
    (setq map (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng8-1f-0.05.pgm") 50 -8000 -70000 :name "/eng8/1f/floor-plane"))

    ;; 1F
    (make-cascoords-tree
     map
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

    (setq scene
	  (instance nav-scene-model :init :name "/eng8/1f"
		    :objects (append (list map) (send map :descendants))))
    (send scene :map map)
    scene ))

(defun make-eng8-6f-scene ()
  (let* (scene map floor-names rooms spots objects room602 room610)
    (setq map (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng8-6f-0.05.pgm") 50 -8000 -70000 :name "/eng8/6f/floor-plane"))

    ;; 6F
    (setq room610 (instance room610-scene :init :name "610")
          room602 (instance room602-scene :init :name "602"))
    (setf (get room602 :knowrob-type) "OfficeRoom")
    (setf (get room610 :knowrob-type) "OfficeRoom")
      ;;
    (send room602 :locate (float-vector 0 0 0))
    (send room610 :locate (float-vector 100 -13678 0))
    ;;
    (dolist (obj (send room602 :objects)) (send room602 :assoc obj))
    (dolist (obj (send room610 :objects)) (send room610 :assoc obj))
    (send map :assoc room602)
    (send map :assoc room610)

    (make-cascoords-tree
     map
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
	    :name "/eng8/6f/elevator_inside_panel")

     ;;start_map_eng8.launch 0  -13.678 0
     '(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
       :pos #f(1050.0 -13678.0 0.0)
       :name "/eng8/6f/room610-base")
     `(:rpy ,(float-vector pi/2 0 0)
       :pos ,(float-vector 5850 -14500 0)
       :name "/eng8/6f/room610-table-front")
     `(:rpy ,(float-vector pi/2 0 0)
       :pos ,(float-vector 4500 -12500 0)
       :name "/eng8/6f/room610-fridge-front")
     `(:rpy ,(float-vector pi/2 0 0)
       :pos ,(float-vector 7000 -12500 0)
       :name "/eng8/6f/room610-laundry-front")
     `(:rpy ,(float-vector 0 0 0)
       :pos ,(float-vector 7500 -15000 0)
       :name "/eng8/6f/room610-drawer-front")
     `(:rpy ,(float-vector 0 0 0)
       :pos ,(float-vector 7300 -12700 0)
       :name "/eng8/6f/room610-dishwasher-front")
     `(:rpy ,(float-vector 0 0 0)
       :pos ,(float-vector 7300 -14100 0)
       :name "/eng8/6f/room610-kitchen-front")
     `(:rpy ,(float-vector pi/2 0 0)
       :pos ,(float-vector 5800 -12200 0)
       :name "/eng8/6f/room610-shelf-front")
     `(:rpy ,(float-vector pi/2 0 0)
       :pos ,(float-vector 6200 -12200 0)
       :name "/eng8/6f/room610-microwave-front")
     `(:rpy ,(float-vector -pi/2 0 0)
       :pos ,(float-vector 2750 -14900 0)
       :name "/eng8/6f/room610-counter-front")
     `(:rpy ,(float-vector -pi/2 0 0)
       :pos ,(float-vector 3850 -15000 0)
       :name "/eng8/6f/room610-cabinet-front")
     )
    
    (setq scene
	  (instance nav-scene-model :init :name "/eng8/6f"
		    :objects (append (list map) (send map :descendants))))
    (send scene :map map)
    scene ))


(defun make-eng8-scene ()
  (let* (scene maps floor-names spots objects)
    (setq maps (list (make-eng8-b1f-scene)
		     (make-eng8-1f-scene)
		     (make-eng8-6f-scene)))

    ;; this is not true
    (dotimes (i (length maps))
      (send (elt maps i) :translate (float-vector 0 0 (* i 10000))))


    (setq scene
	  (instance
	   building-model :init :name "/eng8"
	   :objects
	   (append
	    maps
	    (list
	     (elevator_call_panel :name "elevator_call_panel")
	     (elevator_inside_panel :name "elevator_inside_panel"))
	    )))

    ;; scene -> map
    ;;(send scene :assoc map)
    ;;(send scene :floors floor-names) ;; set floor information

    ;; visualize region
    (send-all (send-all maps :map) :set-color #f(0 1 0 0.3))

    (dolist (obj (send scene :objects))
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
