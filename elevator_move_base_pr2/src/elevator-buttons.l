(load "package://eusmodel_sift_detector/euslisp/all-objects-irtmodel.l")

(defun make-elevator-call-panel (floor-name)
  (let ((obj (instance elevator_call_panel-object :init)))
    ;; assoc coordinates
    (dolist (co (list (make-cascoords :name "up" :pos #f(0 -50 20))
		      (make-cascoords :name "down" :pos #f(0 -50 -45))))
      (send co :rotate pi :z)
      (send obj :assoc co))
    (send obj :put :prior-coords
	 (list (make-cascoords :name floor-name :pos #f(3100 -29900 920) :rpy #f(1.5706 0 0))))
    obj))

(defun make-elevator-inside-panel ()
  (let ((obj (instance elevator_inside_panel-object :init)) disp)
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

    (send obj :put :prior-coords
	  (list (make-cascoords :name "/eng8_1f" :pos #f(4850 -31550 1000) :rpy #f(3.1416 0 0))
		(make-cascoords :name "/eng8_6f" :pos #f(4850 -31550 1000) :rpy #f(3.1416 0 0))))
    obj))


(defun make-eng8-map ()
  (let ((map (make-cube (* 50 1740) (* 50 1000) 1))
	top-face)
    (send map :name "/eng8")
    (send map :translate-vertices #f(-33500 13000 0)) ;; 87m/2 - 77m, 50m/2 - 12m
    (send map :rotate-vertices pi/2 :z)
    (setq top-face (find-if #'(lambda(f)(equal (send f :id) '(:top)))
			    (send map :faces)))
    (send map :paste-texture-to-face top-face :file
	  (ros::resolve-ros-path "package://jsk_maps/8goukan.pgm"))
    (dolist (casco (list (make-cascoords :name "/eng8_1f" :pos #f(-30000 0 0))
			 (make-cascoords :name "/eng8_6f" :pos #f(     0 0 0))))
      (send map :assoc casco))
    map
    ))


(load "~/prog/euslib/irteus_proposals/scene.l")
(defun make-eng8-scene nil
  (let* ((scene (instance scene-model :init
			  :name "/eng8"
			  :objects (list
				    (make-elevator-call-panel "/eng8_1f")
				    (make-elevator-call-panel "/eng8_6f")
				    (make-elevator-inside-panel)
				    (make-eng8-map)
				    ;; for test
				    ;;(instance twinings-object :init)
				    ;;(instance coffee-filter-object :init)
				    )))
	 (map (car (send scene :find-object (send scene :name)))))
    ;; name from class-name
    (dolist (obj (send scene :objects))
      (let ((class-name (string-downcase (send (class obj) :name))))
	(when (and (null (send obj :name)) (substringp "-object" class-name))
	  (send obj :name (format nil "/~a" (subseq class-name 0 (- (length class-name) (length "-object"))))))
	(when (not (eq (char (send obj :name) 0) #\/))
	  (send obj :name (format nil "/~A" (send obj :name))))
	))
    ;; scene -> map
    (send scene :assoc map)
    ;; map -> obj
    (dolist (obj (remove map (send scene :objects) :count 1))
      (when (send obj :get :prior-coords)
	(let* ((prior (car (send obj :get :prior-coords)))
	       (org-coords
		(find-if #'(lambda(c)(string= (send c :name) (send prior :name)))
			 (flatten (send scene :inheritance)))))
	  (if org-coords
	      (send obj :move-to (car (send obj :get :prior-coords)) org-coords))))
      (send map :assoc obj))
    scene))

;;
(defun update-scene-by-tf (scene &optional (tm (ros::time 0)))
  (let* ((objects (flatten (send scene :inheritance)))
	 (fixed-frame (send scene :name)) ;; ok ?
	 (fixed-obj (car (send scene :find-object fixed-frame)))
	 (updated nil))
    (if (not fixed-obj) (return-from update-scene-by-tf nil))
    (dolist (obj objects)
      (when (and (stringp (send obj :name))
	     (send *tfl* :can-transform fixed-frame (send obj :name) tm))
	(let ((trans (send *tfl* :lookup-transform
			   fixed-frame (send obj :name) tm)))
	  (send obj :move-to trans fixed-obj)
	  (push obj updated)
	  ;;(ros::ros-info "trans ~A is ~A" obj trans)
      )))
    updated
    ))

;; patch for irt_proposals/scene.l
(defmethod body
  (:bodies () (list self)))
(defmethod scene-model
  (:bodies () (flatten (send-all objects :bodies))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

#|
(defun make-elevator-call-panel (floor-name)
  (instance posedetectable-panel-object :init
	    (make-elevator-call-panel-body)
            :buttons (list (make-cascoords :name "up" :pos #f(0 -50 20))
                           (make-cascoords :name "down" :pos #f(0 -50 -45)))
	    :prior-coords (list (make-cascoords :name floor-name :pos #f(3100 -29900 920) :rpy #f(1.5706 0 0)))
            ))

(defun make-elevator-inside-panel ()
  (instance posedetectable-panel-object :init
	    (make-elevator-inside-panel-body)
            :buttons (list (make-cascoords :name "B2F" :pos #f(0 -135 -95))
			   (make-cascoords :name "B1F" :pos #f(0 -80 -95))
			   (make-cascoords :name "1F" :pos #f(0 -25 -95))
			   (make-cascoords :name "2F" :pos #f(0 30 -95))
			   (make-cascoords :name "3F" :pos #f(0 85 -95))
                           (make-cascoords :name "4F" :pos #f(0 -25 -30))
                           (make-cascoords :name "5F" :pos #f(0 30 -30))
                           (make-cascoords :name "6F" :pos #f(0 85 -30))
                           (make-cascoords :name "7F" :pos #f(0 140 -30))
			   (make-cascoords :name "open" :pos #f(0 85 35))
			   (make-cascoords :name "close" :pos #f(0 140 35)))
	    :prior-coords (list (make-cascoords :name "/eng8_1f" :pos #f(4850 -31550 1000) :rpy #f(3.1416 0 0))
				(make-cascoords :name "/eng8_6f" :pos #f(4850 -31550 1000) :rpy #f(3.1416 0 0)))
            ))

(defun make-all-objects ()
  (setq *all-objects* (list (make-elevator-call-panel "/eng8_1f")
			    (make-elevator-call-panel "/eng8_6f")
			    (make-elevator-inside-panel)
			    (make-eng8-map)))
  ;; assoc object to map by prior-coords (car)
  (let* ((map (find-object-by-name "/eng8"))
	 (origins (append (list map) (flatten (send map :inheritance)))))
    (dolist (obj *all-objects*)
      (when (derivedp obj posedetectable-panel-object)
	(let ((set-org
	       (find-if #'(lambda(o)(equal (send o :name) (send (car (send obj :prior-coords)) :name))) origins)))
	  (when set-org
	    (send obj :move-to set-org :world)
	    (send obj :move-to (car (send obj :prior-coords)) :local)
	    (send obj :worldpos)
	    (send set-org :assoc obj)
	    )))))
  *all-objects*)

(defun find-object-by-name (name &key (all nil))
  (if all
      (mapcan #'(lambda (o) (if (equal name (send o :name)) (list o))) *all-objects*)
    (find-if #'(lambda (o) (equal name (send o :name))) *all-objects*)))
|#


