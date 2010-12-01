(load "models/elevator_call_panel-object.l")
(load "models/elevator_inside_panel-object.l")

(defun make-elevator-call-panel ()
  (let ((obj (instance elevator_call_panel-object :init)))
    ;; assoc buttons
    (dolist (co (list (make-cascoords :name "up" :pos #f(0 -50 20))
		      (make-cascoords :name "down" :pos #f(0 -50 -45))))
      (send co :rotate pi :z)
      (send obj :assoc co))
    obj))

(defun make-elevator-inside-panel ()
  (let ((obj (instance elevator_inside_panel-object :init)) disp)
    ;; assoc buttons
    (dolist (co (list (make-cascoords :name "B2F" :pos #f(0 -135 -95))
		      (make-cascoords :name "B1F" :pos #f(0 -80 -95))
		      (make-cascoords :name "1F" :pos #f(0 -25 -95))
		      (make-cascoords :name "2F" :pos #f(0 30 -95))
		      (make-cascoords :name "3F" :pos #f(0 85 -95))
		      (make-cascoords :name "4F" :pos #f(0 -25 -30))
		      (make-cascoords :name "5F" :pos #f(0 30 -30))
		      (make-cascoords :name "6F" :pos #f(0 85 -30))
		      (make-cascoords :name "7F" :pos #f(0 140 -30))
		      (make-cascoords :name "open" :pos #f(0 85 35))
		      (make-cascoords :name "close" :pos #f(0 140 35))))
      (send co :rotate pi :z)
      (send obj :assoc co))
    (send obj :assoc (make-cube 1 105 40
				:name "/elevator_number"
				:pos #f(0 115 90)))
    obj))

(defun make-eng8-map ()
  (let ((map (make-cube (* 50 1740) (* 50 1000) 1))
	mag-img
	top-face)
    (send map :name "/eng8")
    (send map :translate-vertices #f(-33500 13000 0)) ;; 87m/2 - 77m, 50m/2 - 12m
    (send map :rotate-vertices pi/2 :z)
    (setq top-face (find-if #'(lambda(f)(equal (send f :id) '(:top)))
			    (send map :faces)))
    (setq map-img (read-image-file (ros::resolve-ros-path "package://jsk_maps/eng8.pgm")))
    (setf (get map-img :texture-max-width) 1000)
    (setf (get map-img :texture-max-height) 1740)
    (send map :paste-texture-to-face top-face :image map-img)
    (dolist (casco (list (make-cascoords :name "/eng8/1f" :pos #f(-30000 0 0))
			 (make-cascoords :name "/eng8/6f" :pos #f(     0 0 0))))
      (send map :assoc casco))
    map
    ))

(defun make-eng8-scene ()
  (let* ((map (make-eng8-map))
	 (scene
	  (instance
	   scene-model
	   :init
	   :name "/eng8"
	   :objects
	   (list
	    map
	    (make-elevator-call-panel)
	    (make-elevator-inside-panel)
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
	    (make-cascoords :pos #f(-26900 -29900 920) :rpy #f(1.5706 0 0)
			    :name "/eng8/1f/elevator_call_panel")
	    (make-cascoords :pos #f(-25150 -31550 1000) :rpy #f(3.1416 0 0)
			    :name "/eng8/1f/elevator_inside_panel")
	    (make-cascoords :pos #f(3100 -29900 920) :rpy #f(1.5706 0 0)
			    :name "/eng8/6f/elevator_call_panel")
	    (make-cascoords :pos #f(4850 -31550 1000) :rpy #f(3.1416 0 0)
			    :name "/eng8/6f/elevator_inside_panel")
	    ))))
    ;; scene -> map
    (send scene :assoc map)

    (dolist (obj (send scene :objects))
      ;; map -> obj
      (unless (eq obj map)
	(send map :assoc obj))
      ;; first charactor of obj-name must be slash
      (when (not (eq (char (send obj :name) 0) #\/))
	(send obj :name (format nil "/~A" (send obj :name)))))
    scene))

(defun check-current-floor (&optional (pose (get-robot-pose)))
  (cond ((< (elt (send pose :pos) 0) -5000) "/eng8/1f")
	(t "/eng8/6f")))

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
