(load "package://euslib/irteus_proposals/scene.l")
(load "package://jsk_maps/euslisp/eng8-map.l")
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

(defun make-eng8-scene ()
  (let* ((map (make-eng8-map-map))
	 (scene
	  (instance
	   scene-model
	   :init
	   :name "/eng8"
	   :objects
	   (append
	    (list
	     map
	     (make-elevator-call-panel)
	     (make-elevator-inside-panel))
	    (flatten (send map :inheritance))
	    ))))
    ;; scene -> map
    (send scene :assoc map)

    (dolist (obj (send scene :objects))
      ;; map -> unassoc obj
      (unless (find obj (flatten (send scene :inheritance)))
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
