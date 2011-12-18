(load "package://jsk_maps/src/building-model.l")

(load "models/elevator_call_panel_eng2-object.l")
(load "models/elevator_inside_panel_eng2-object.l")

(load "eng2-rooms.l")

(defun push-knowrob-properties (subj prop-name obj)
  (when (and subj obj)
	(push (cons prop-name obj) (get subj :knowrob-properties))))

(defun make-eng2-2f-scene ()
  (let* (scene map floor-names rooms spots objects)
    (setq map (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng2-2f-0.05.pgm") 50 -13000 -48500 :name "/eng2/2f/floor-plane"))

    ;; 2f
    (let* ((2rms (eng2-2f-rooms)) trans
	   (2f-o map))
	  (setq trans (send (make-coords) :transformation 2f-o))
	  (dolist (rm 2rms)
	    (send rm :name (concatenate string "/eng2/2f/" (send rm :name)))
	    (send 2f-o :assoc rm)
	    (send rm :transform trans :world)))
    (make-cascoords-tree
     map
     `(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 3100.0 -31250.0 0.0)
	    :name "/eng2/2f/elevator_call_panel-front")
     `(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 2000.0 -33800.0 0.0)
	    :name "/eng2/2f/elevator_inside_panel-front")
     `(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 2000.0 -31000.0 0.0)
	    :name "/eng2/2f/elevator-outside")
     `(:pos ,(float-vector 23000.0 -7100.0 0.0) :rpy #f(1.5706 0 0)
	    :name "/eng2/2f/subway-front")
     `(:pos ,(float-vector 25700.0 -6900.0 0.0) :rpy #f(1.5706 0 0)
	    :name "/eng2/2f/subway-register")
     `(:pos ,(float-vector 25000.0 -9400.0 0.0) :rpy #f(1.5706 0 0)
	    :name "/eng2/2f/subway-lunchset")
     `(:pos ,(float-vector 3200.0 -31950 920) :rpy #f(1.5706 0 0)
	    :name "/eng2/2f/elevator_call_panel")
     `(:pos ,(float-vector 2900 -33850 1000) :rpy #f(3.1416 0 0)
	    :name "/eng2/2f/elevator_inside_panel"))

    (setq scene (instance nav-scene-model :init :name "/eng2/2f" :objects (append (list map) (send map :descendants))))
    (send scene :map map)
    scene ))

(defun make-eng2-3f-scene ()
  (let* (scene map floor-names rooms spots objects)
    (setq map (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng2-3f-0.05.pgm") 50 -13000 -48500 :name "/eng2/3f/floor-plane"))

    ;; 3F
    (make-cascoords-tree
     (find "/eng2/3f" (send map :descendants)
	   :test #'string= :key #'(lambda(x)(send x :name)))
     `(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector -2950 -32650 0)
	    :name "/eng2/3f/room-31A-front")
     `(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector -7646 -32526 0)
	    :name "/eng2/3f/room-31A-center")
     `(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector -8667 -35244 0)
	    :name "/eng2/3f/room-31A-inaba-front")
     `(:rot #2f((0.726141 0.68746 0.0) (-0.68746 0.726141 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector -6128 -43994 0)
	    :name "/eng2/3f/room-31A-inoue-front")
     `(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector -2393 -28345 0)
	    :name "/eng2/3f/cross-road")
     `(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector -2590 -44905 0)
	    :name "/eng2/3f/small-kitchen"))

    (setq scene (instance nav-scene-model :init :name "/eng2/3f" :objects (append (list map) (send map :descendants))))
    (send scene :map map)
    scene ))


(defun make-eng2-7f-scene ()
  (let* (scene map floor-names rooms spots objects)
    (setq map (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng2-7f-0.05.pgm") 50 -13000 -48500 :name "/eng2/7f/floor-plane"))

    ;; 7f
    (let* ((7rms (eng2-7f-rooms)) trans
	   (7f-o map)
	   (73b1 (find "73b1" 7rms
		       :test #'string= :key #'(lambda(x)(send x :name)))))
      (setq trans (send 73b1 :transformation 7f-o))
      (dolist (rm 7rms)
	(send rm :name (concatenate string "/eng2/7f/" (send rm :name)))
	(send 7f-o :assoc rm)
	(send rm :transform trans :world)
	(mapc #'(lambda(o)(unless (send o :parent)
			    (send o :transform trans :world)))
	      (send-if-found nil rm :objects))
	))

    ;;
    (make-cascoords-tree
     map
     `(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 3100.0 -31250.0 0.0)
	    :name "/eng2/7f/elevator_call_panel-front")
     `(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 2000.0 -33800.0 0.0)
	    :name "/eng2/7f/elevator_inside_panel-front")
     `(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 2000.0 -31000.0 0.0)
	    :name "/eng2/7f/elevator-outside")
     `(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector -1500.0 9800.0 0.0)
	    :name "/eng2/7f/room73A4-front")
     `(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector -1150.0 6300.0 0.0)
	    :name "/eng2/7f/room73A3-front")
     `(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector -3250.0 4900.0 0.0)
	    :name "/eng2/7f/room73A3-base")
     `(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector -1500.0 2900.0 0.0)
	    :name "/eng2/7f/room73A2-front")
     `(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector -1500.0 -500.0 0.0)
	    :name "/eng2/7f/room73A1-front")
     `(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 3400.0 -26600.0 0.0)
	    :name "/eng2/7f/combustible-trashbox-front")
     `(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 5000.0 -26600.0 0.0)
	    :name "/eng2/7f/non-combustible-trashbox-front")
     `(:rot #2f((0.96 0.26 0.0) (-0.26 0.96 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 5000.0 -2400.0 0.0)
	    :name "/eng2/7f/room73B1-nakanishi")
     `(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector -2900 -4750 0.0)
	    :name "/eng2/7f/mini-kitchen-A-inside")
     `(:rot #2f((0.52 0.92 0.0) (-0.92 0.52 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 3300.0 -1900.0 0.0)
	    :name "/eng2/7f/room73B1-kurotobi")
     `(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector -5500.0 4800.0 0.0)
	    :name "/eng2/7f/room73A3-printer-front")
     `(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 2850 8600 0.0)
	    :name "/eng2/7f/room73B2-coe-shelf")
     `(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 1300 8900 0.0)
	    :name "/eng2/7f/room73B2-kitchen-drawer")
     `(:pos (float-vector 940 6950 0) :rpy (float-vector -0.580 0 0)
	    :name "/eng2/7f/room73B2-fridge-front")
     ;; `(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
     ;;        :pos ,(float-vector 1300 8974 0.0)
     ;;        :name "/eng2/7f/room73B2-kitchenboard")
     ;; `(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
     ;;        :pos ,(float-vector 1423 8538 0.0)
     ;;        :name "/eng2/7f/room73B2-kitchendrawer")
     ;; `(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
     ;;        :pos ,(float-vector 3679 6549 0.0)
     ;;        :name "/eng2/7f/room73B2-besides-bed")
     ;; `(:rot #2f((0.893829 0.44758 0.0) (-0.44758 0.893829 0.0) (0.0 0.0 0.999629))
     ;;        :pos ,(float-vector 1086 6992 0.0)
     ;;        :name "/eng2/7f/room73B2-refridge")
     ;; `(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
     ;;        :pos ,(float-vector 4084 7095 0.0)
     ;;        :name "/eng2/7f/room73B2-chair")
     `(:pos ,(float-vector 3200 -31950 920) :rpy #f(1.5706 0 0)
	    :name "/eng2/7f/elevator_call_panel")
     `(:pos ,(float-vector 2900 -33800 1000) :rpy #f(3.1416 0 0)
	    :name "/eng2/7f/elevator_inside_panel"))

    (setq scene (instance nav-scene-model :init :name "/eng2/7f" :objects (append (list map) (send map :descendants))))
    (send scene :map map)
    scene ))


(defun make-eng2-8f-scene ()
  (let* (scene map floor-names rooms spots objects)
    (setq map (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng2-8f-0.05.pgm") 50 -13000 -48500 :name "/eng2/8f/floor-plane"))

    ;; 8F
    (let* ((8rms (eng2-8f-rooms)) trans
	   (8f-o map)
	   (83b1 (find "83b1" 8rms
		       :test #'string= :key #'(lambda(x)(send x :name)))))
      (setq trans (send 83b1 :transformation 8f-o))
      (dolist (rm 8rms)
	(send rm :name (concatenate string "/eng2/8f/" (send rm :name)))
	(send 8f-o :assoc rm)
	(send rm :transform trans :world)
	(mapc #'(lambda(o)(unless (send o :parent)
			    (send o :transform trans :world)))
	      (send-if-found nil rm :objects))
	))
    ;;
    (make-cascoords-tree
     map
     `(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector -2900 -4650 0.0)
	    :name "/eng2/8f/mini-kitchen-A-inside")
     `(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 3100.0 -31250.0 0.0)
	    :name "/eng2/8f/elevator_call_panel-front")
     `(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 2000.0 -33800.0 0.0)
	    :name "/eng2/8f/elevator_inside_panel-front")
     `(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos ,(float-vector 2000.0 -31000.0 0.0)
	    :name "/eng2/8f/elevator-outside")
     `(:pos ,(float-vector 3200 -31950 920) :rpy #f(1.5706 0 0)
	    :name "/eng2/8f/elevator_call_panel")
     `(:pos ,(float-vector 2900 -33800 1000) :rpy #f(3.1416 0 0)
	    :name "/eng2/8f/elevator_inside_panel"))

    (setq scene (instance nav-scene-model :init :name "/eng2/8f" :objects (append (list map) (send map :descendants))))
    (send scene :map map)
    scene ))


(defun make-eng2-scene ()
  (let* (scene maps floor-names rooms spots objects)
    (setq maps (list (make-eng2-2f-scene)
		     (make-eng2-3f-scene)
		     (make-eng2-7f-scene)
		     (make-eng2-8f-scene)))

    ;; this is not true
    (dotimes (i (length maps))
      (send (elt maps i) :translate (float-vector 0 0 (* i 10000))))

    (setq rooms (remove-if-not #'(lambda(x)(derivedp x scene-model)) (flatten (send-all maps :objects))))

    ;; elevator bounding box
    (dolist (map maps)
      (let ((elevator (make-cube 2000 1800 2600 :name (format nil "~a/elevator" (send map :name)))))
	(send elevator :move-to (send map :copy-worldcoords))
	(send elevator :translate #f(1700 -33900 1300))
	(send map :assoc elevator)
	(send elevator :set-color #f(1 0 0 0.4))
	(push elevator objects)
	(setf (get elevator :knowrob-type) "Elevator")
	))

    (push (elevator_call_panel_eng2 :name "elevator_call_panel") objects)
    (push (elevator_inside_panel_eng2 :name "elevator_inside_panel") objects)

    (setq scene
	  (instance
	   building-model :init :name "/eng2"
	   :objects
	   (append maps objects)
	   ))

    ;; scene -> map
    ;;(send scene :assoc map)
    ;;(send scene :floors floor-names) ;; set floor information
    ;;(send scene :rooms rooms)

    ;; visualize region
    (send-all (send-all maps :map) :set-color #f(0 1 0 0.3))
    (send-all rooms :translate #f(0 0 2))

    (dolist (obj (send scene :objects))
      ;; first charactor of obj-name must be slash
      (when (not (eq (char (string (send obj :name)) 0) #\/))
	(send obj :name (format nil "/~A" (send obj :name)))))

    ;; move panels to initial pose
    (send (send scene :object "/elevator_call_panel") :move-to
	  (send (send scene :spot "/eng2/7f/elevator_call_panel") :copy-worldcoords) :world)
    (send (send scene :object "/elevator_inside_panel") :move-to
	  (send (send scene :spot "/eng2/7f/elevator_inside_panel") :copy-worldcoords) :world)
    (send (send scene :object "/eng2/7f") :assoc
	  (send scene :object "/elevator_call_panel"))
    (send (send scene :object "/eng2/7f") :assoc
	  (send scene :object "/elevator_inside_panel"))

    ;; add scene type for knowrob
    (dolist (fl rooms)
      (when (not (get fl :knowrob-type))
	(setf (get fl :knowrob-type) "RoomInAConstruction")))
    (dolist (fl (send scene :floors))
      (setf (get fl :knowrob-type) "AboveGroundLevelInAConstruction"))
    (setf (get scene :knowrob-type) "Building")

    (push-knowrob-properties (send scene :object "/eng2/7f/room73B2-table-front")
			     "inFrontOf-Generally"
			     (send scene :object "room73b2-x-leg-desk"))
    (push-knowrob-properties (send scene :object "/eng2/7f/room73B2-front-of-tv")
			     "inFrontOf-Generally"
			     (send scene :object "room73b2-sharp-52-aquostv"))
    (push-knowrob-properties (send scene :object "/eng2/7f/room73A1-front")
			     "inFrontOf-Generally"
			     (send scene :object "/eng2/7f/73a1"))
    (push-knowrob-properties (send scene :object "/eng2/7f/room73A2-front")
			     "inFrontOf-Generally"
			     (send scene :object "/eng2/7f/73a2"))
    (push-knowrob-properties (send scene :object "/eng2/7f/room73A3-front")
			     "inFrontOf-Generally"
			     (send scene :object "/eng2/7f/73a3"))
    (push-knowrob-properties (send scene :object "/eng2/7f/room73A4-front")
			     "inFrontOf-Generally"
			     (send scene :object "/eng2/7f/73a4"))
    (push-knowrob-properties (send scene :object "/eng2/2f/elevator-outside")
			     "inFrontOf-Generally"
			     (send scene :object "/eng2/2f/elevator"))
    (push-knowrob-properties (send scene :object "/eng2/7f/elevator-outside")
			     "inFrontOf-Generally"
			     (send scene :object "/eng2/7f/elevator"))
    (push-knowrob-properties (send scene :object "/eng2/8f/elevator-outside")
			     "inFrontOf-Generally"
			     (send scene :object "/eng2/8f/elevator"))

    scene))

;; example
;; (load "package://pr2eus/pr2-interface.l")
;; (ros::roseus "eng8_navigation")
;; (defvar *tfl* (instance ros::transform-listener :init))
;; (setq *ri* (instance pr2-interface :init))
;; (objects (list (setq *eng8-scene* (make-eng8-scene))))
;; (move-to (car (send *eng8-scene* :spot "/eng8/6f/room602-front")))
;;
;; debug
#|
(defun move-to-test (&optional (spot "/eng8/6f/room602-base"))
  (load "eng8-map-scene.l")
  (objects (list (setq *eng8-scene*  (make-eng8-scene))))
  (send *irtviewer* :look-all (make-cube 1000 1000 10000))
  (send (car (send *eng8-scene* :object spot)) :draw-on :flush t :size 1000 :color #f(1 0 0))
  (move-to (car (send *eng8-scene* :object spot)))
  )
|#
