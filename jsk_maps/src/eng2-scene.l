(load "package://jsk_maps/src/plane-building-model.l")
(load "package://jsk_maps/euslisp/eng2-map.l")

(load "models/elevator_call_panel_eng2-object.l")
(load "models/elevator_inside_panel_eng2-object.l")

(load "eng2-rooms.l")

(defun push-knowrob-properties (subj prop-name obj)
  (when (and subj obj)
	(push (cons prop-name obj) (get subj :knowrob-properties))))

(defun make-eng2-scene ()
  (let* (scene (map (make-eng2-map-map)) floor-names rooms spots objects)
    (setq floor-names '("/eng2/2f" "/eng2/3f" "/eng2/7f" "/eng2/8f"))

    ;; 2f
    (let* ((2rms (eng2-2f-rooms)) trans
		   (2f-o (find "/eng2/2f" (send map :descendants)
					   :test #'string= :key #'(lambda(x)(send x :name)))))
	  (setq trans (send (make-coords) :transformation 2f-o))
      (dolist (rm 2rms)
		(send rm :name (concatenate string "/eng2/2f/" (send rm :name)))
		(send 2f-o :assoc rm)
		(send rm :transform trans :world))
      (setq rooms (append rooms 2rms)))
    (make-cascoords-tree
     (find "/eng2/2f" (send map :descendants)
	   :test #'string= :key #'(lambda(x)(send x :name)))
     '(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(3100.0 -31250.0 0.0)
	    :name "/eng2/2f/elevator_call_panel-front")
     '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(2100.0 -33800.0 0.0)
	    :name "/eng2/2f/elevator_inside_panel-front")
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(2000.0 -31000.0 0.0)
	    :name "/eng2/2f/elevator-outside")
     '(:pos #f(23000.0 -7100.0 0.0) :rpy #f(1.5706 0 0)
	    :name "/eng2/2f/subway-front")
     '(:pos #f(25700.0 -7100.0 0.0) :rpy #f(1.5706 0 0)
	    :name "/eng2/2f/subway-register")
     '( :pos #f(25000.0 -9400.0 0.0) :rpy #f(1.5706 0 0)
	    :name "/eng2/2f/subway-lunchset")
     '(:pos #f(3200.0 -31950 920) :rpy #f(1.5706 0 0)
	    :name "/eng2/2f/elevator_call_panel")
     '(:pos #f(2900 -33750 1000) :rpy #f(3.1416 0 0)
	    :name "/eng2/2f/elevator_inside_panel"))
    ;; 3F
    (make-cascoords-tree
     (find "/eng2/3f" (send map :descendants)
	   :test #'string= :key #'(lambda(x)(send x :name)))
     '(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(-2950 -32650 0)
	    :name "/eng2/3f/room-31A-front")
     '(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(-7646 -32526 0)
	    :name "/eng2/3f/room-31A-center")
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(-8667 -35244 0)
	    :name "/eng2/3f/room-31A-inaba-front")
     '(:rot #2f((0.726141 0.68746 0.0) (-0.68746 0.726141 0.0) (0.0 0.0 1.0))
	    :pos #f(-6128 -43994 0)
	    :name "/eng2/3f/room-31A-inoue-front")
     '(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(-2393 -28345 0)
	    :name "/eng2/3f/cross-road")
     '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(-2590 -44905 0)
	    :name "/eng2/3f/small-kitchen"))
    ;; 7f
    (let* ((7rms (eng2-7f-rooms)) trans
		   (7f-o (find "/eng2/7f" (send map :descendants)
					   :test #'string= :key #'(lambda(x)(send x :name))))
		   (73b1 (find "73b1" 7rms
					   :test #'string= :key #'(lambda(x)(send x :name)))))
	  (setq trans (send 73b1 :transformation 7f-o))
      (dolist (rm 7rms)
		(send rm :name (concatenate string "/eng2/7f/" (send rm :name)))
		(send 7f-o :assoc rm)
		(send rm :transform trans :world))
      (setq rooms (append rooms 7rms)))
    (make-cascoords-tree
     (find "/eng2/7f" (send map :descendants)
	   :test #'string= :key #'(lambda(x)(send x :name)))
     '(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(3100.0 -31250.0 0.0)
	    :name "/eng2/7f/elevator_call_panel-front")
     '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(2100.0 -33800.0 0.0)
	    :name "/eng2/7f/elevator_inside_panel-front")
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(2000.0 -31000.0 0.0)
	    :name "/eng2/7f/elevator-outside")
     '(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(-1500.0 9800.0 0.0)
	    :name "/eng2/7f/room73A4-front")
     '(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(-1150.0 6300.0 0.0)
	    :name "/eng2/7f/room73A3-front")
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(-3250.0 4900.0 0.0)
	    :name "/eng2/7f/room73A3-base")
     '(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(-1500.0 2900.0 0.0)
	    :name "/eng2/7f/room73A2-front")
     '(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(-1500.0 -500.0 0.0)
	    :name "/eng2/7f/room73A1-front")
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(3700.0 8600.0 0.0)
	    :name "/eng2/7f/room73B2-front-of-tv")
     '(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(2293.0 8883.0 0.0)
	    :name "/eng2/7f/room73B2-front-kitchen-table")
     '(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(5150.0 9080.0 0.0)
	    :name "/eng2/7f/room73B2-table-front")
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(5155.0 6910.0 0.0)
	    :name "/eng2/7f/room73B2-table-back")
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(3400.0 -26600.0 0.0)
	    :name "/eng2/7f/combustible-trashbox-front")
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(5000.0 -26600.0 0.0)
	    :name "/eng2/7f/non-combustible-trashbox-front")
     '(:rot #2f((0.96 0.26 0.0) (-0.26 0.96 0.0) (0.0 0.0 1.0))
	    :pos #f(5000.0 -2400.0 0.0)
	    :name "/eng2/7f/room73B1-nakanishi")
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
            ;;:pos #f(-2838 -4709 0.0)
	    :pos #f(-2900 -4750 0.0)
	    :name "/eng2/7f/mini-kitchen-A-inside")
     '(:rot #2f((0.52 0.92 0.0) (-0.92 0.52 0.0) (0.0 0.0 1.0))
	    :pos #f(3300.0 -1900.0 0.0)
	    :name "/eng2/7f/room73B1-kurotobi")
     '(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(-5500.0 4800.0 0.0)
	    :name "/eng2/7f/room73A3-printer-front")
     '(:pos #f(3200 -31950 920) :rpy #f(1.5706 0 0)
	    :name "/eng2/7f/elevator_call_panel")
     '(:pos #f(2900 -33750 1000) :rpy #f(3.1416 0 0)
	    :name "/eng2/7f/elevator_inside_panel"))
    ;; 8F
    (let* ((8rms (eng2-8f-rooms)) trans
	   (8f-o (find "/eng2/8f" (send map :descendants)
		       :test #'string= :key #'(lambda(x)(send x :name))))
	   (83b1 (find "83b1" 8rms
		       :test #'string= :key #'(lambda(x)(send x :name)))))
	  (setq trans (send 83b1 :transformation 8f-o))
      (dolist (rm 8rms)
		(send rm :name (concatenate string "/eng2/8f/" (send rm :name)))
		(send 8f-o :assoc rm)
		(send rm :transform trans :world))
      (setq rooms (append rooms 8rms)))
    (make-cascoords-tree
     (find "/eng2/8f" (send map :descendants)
	   :test #'string= :key #'(lambda(x)(send x :name)))
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(-2900 -4750 0.0)
	    :name "/eng2/8f/mini-kitchen-A-inside")
     '(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(3100.0 -31250.0 0.0)
	    :name "/eng2/8f/elevator_call_panel-front")
     '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(2100.0 -33800.0 0.0)
	    :name "/eng2/8f/elevator_inside_panel-front")
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(2000.0 -31000.0 0.0)
	    :name "/eng2/8f/elevator-outside")
     '(:pos #f(3200 -31950 920) :rpy #f(1.5706 0 0)
	    :name "/eng2/8f/elevator_call_panel")
     '(:pos #f(2900 -33750 1000) :rpy #f(3.1416 0 0)
	    :name "/eng2/8f/elevator_inside_panel"))

	;; elevator bounding box
	(dolist (nm floor-names)
	  (let ((elevator (make-cube 2000 1800 2600 :name (format nil "~a/elevator" nm)))
			(floor-origin (find nm (send map :descendants)
								:test #'string= :key #'(lambda(x)(send x :name)))))
		(send elevator :move-to (send floor-origin :copy-worldcoords))
		(send elevator :translate #f(1700 -33900 1300))
		(send floor-origin :assoc elevator)
		(send elevator :set-color #f(1 0 0 0.4))
;		(push elevator objects)
		(setf (get elevator :knowrob-type) "Elevator")
		))

	(push (elevator_call_panel_eng2 :name "elevator_call_panel") objects)
	(push (elevator_inside_panel_eng2 :name "elevator_inside_panel") objects)

    ;; TODO smart
    (setq map1 (list map)
	  map2 (flatten (send-all map1 :descendants)) ;; floor
	  map3 (flatten (send-all map2 :descendants)));; rooms or...

    (setq scene
	  (instance
	   plane-building-model :init :name "/eng2"
	   :objects
	   (append map1 map2 map3 objects)
	   ))

    ;; scene -> map
    (send scene :assoc map)
    (send scene :floors floor-names) ;; set floor information
    (send scene :rooms rooms)

    ;; visualize region
    (send-all (send scene :floors) :set-color #f(0 1 0 0.3))
    (send-all rooms :translate #f(0 0 2))

    (dolist (obj (send scene :objects))
      ;; first charactor of obj-name must be slash
      (when (not (eq (char (string (send obj :name)) 0) #\/))
	(send obj :name (format nil "/~A" (send obj :name)))))

	;; move panels to initial pose
	(send (car (send scene :find-object "/elevator_call_panel")) :move-to
		  (send scene :spot "/eng2/7f/elevator_call_panel") :world)
	(send (car (send scene :find-object "/elevator_inside_panel")) :move-to
		  (send scene :spot "/eng2/7f/elevator_inside_panel") :world)
	(send (car (send scene :find-object "/eng2/7f")) :assoc
		  (car (send scene :find-object "/elevator_call_panel")))
	(send (car (send scene :find-object "/eng2/7f")) :assoc
		  (car (send scene :find-object "/elevator_inside_panel")))


	;; add type for knowrob
    (dolist (fl (send scene :rooms))
	  (when (not (get fl :knowrob-type))
		(setf (get fl :knowrob-type) "RoomInAConstruction")))
    (dolist (fl (send scene :floors))
	  (setf (get fl :knowrob-type) "AboveGroundLevelInAConstruction"))
	(setf (get scene :knowrob-type) "Building")
	(setf (get map :knowrob-type) :ignore) ;; not convert this

	(let ((73b2 (car (send scene :find-object "/eng2/7f/73b2"))))
	  (push-knowrob-properties (car (send scene :find-object "/eng2/7f/room73B2-table-front"))
							   "inFrontOf-Generally"
							   (car (send 73b2 :find-object "room73b2-x-leg-desk")))
	  (push-knowrob-properties (car (send scene :find-object "/eng2/7f/room73B2-front-of-tv"))
							   "inFrontOf-Generally"
							   (car (send 73b2 :find-object "room73b2-sharp-52-aquostv")))
	  )
	(push-knowrob-properties (car (send scene :find-object "/eng2/7f/room73A1-front"))
							 "inFrontOf-Generally"
							 (car (send scene :find-object "/eng2/7f/73a1")))
	(push-knowrob-properties (car (send scene :find-object "/eng2/7f/room73A2-front"))
							 "inFrontOf-Generally"
							 (car (send scene :find-object "/eng2/7f/73a2")))
	(push-knowrob-properties (car (send scene :find-object "/eng2/7f/room73A3-front"))
							 "inFrontOf-Generally"
							 (car (send scene :find-object "/eng2/7f/73a3")))
	(push-knowrob-properties (car (send scene :find-object "/eng2/7f/room73A4-front"))
							 "inFrontOf-Generally"
							 (car (send scene :find-object "/eng2/7f/73a4")))
	(push-knowrob-properties (car (send scene :find-object "/eng2/7f/room73B1-front"))
							 "inFrontOf-Generally"
							 (car (send scene :find-object "/eng2/7f/73b1")))
	(push-knowrob-properties (car (send scene :find-object "/eng2/7f/room73B2-front"))
							 "inFrontOf-Generally"
							 (car (send scene :find-object "/eng2/7f/73b2")))
	(push-knowrob-properties (car (send scene :find-object "/eng2/2f/elevator-outside"))
							 "inFrontOf-Generally"
							 (car (send scene :find-object "/eng2/2f/elevator")))
	(push-knowrob-properties (car (send scene :find-object "/eng2/3f/elevator-outside"))
							 "inFrontOf-Generally"
							 (car (send scene :find-object "/eng2/3f/elevator")))
	(push-knowrob-properties (car (send scene :find-object "/eng2/7f/elevator-outside"))
							 "inFrontOf-Generally"
							 (car (send scene :find-object "/eng2/7f/elevator")))
	(push-knowrob-properties (car (send scene :find-object "/eng2/8f/elevator-outside"))
							 "inFrontOf-Generally"
							 (car (send scene :find-object "/eng2/8f/elevator")))

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
