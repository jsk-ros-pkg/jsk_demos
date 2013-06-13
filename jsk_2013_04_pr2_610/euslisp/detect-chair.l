#!/usr/bin/env roseus


(load "package://pr2eus/pr2-interface.l")
;;(load "models/room610-object-object.l")
(load "package://jsk_2013_04_pr2_610/pfilter.l")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "posedetection_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "jsk_2013_04_pr2_610")
(defparameter *result-topic* "/narrow_stereo/left/ObjectDetection")
(defparameter *check-topic* "/check_topic")
(defvar *base-frame-id* "/base_footprint")

(ros::roseus "detect_object")

(setq *room610-origin* "/eng8/6f/610")
(setq *target-object* (instance room610-object-object :init))
(setq *ri* (instance pr2-interface :init))
(setq *object-width* 0.37)
(setq *object-height* 0.43)
(setq *error-thred* 0.02)
(setq *object-xy* nil)
(setq *inner-threshold* 0.2)

(setq *detect-pf* nil)
(setq *detect-system-noise*)
(setq *detect-measurement-noise*)


(defclass detect-particle-filter
  :super particle-filter)
(defmethod detect-particle-filter
  (:likelihood
   (x y)
   )
  )


(defun init-particles ()
      (setq *detect-pf* (instance detect-particle-filter :init :state-dim 3
		       :particle-num 200
		       :system-noise *detect-system-noise*
		       :measurement-noise *detect-measurement-noise*
		       :initial-noise 1.0
                       :use-adaptive-particle-num t))
    (send *detect-pf* :particle-num 20)
    (send *detect-pf* :initial-noise 0.1)
	(setq pr2->target (send (send *pr2* :base_laser_link) :transformation *target-object*))
	(steq pr2->target-pos (send pr2->target :worldpos))
	(setq pr2->target-rot (send pr2->target :rpy-angle))
    (send *detect-pf* :initial-state (float-vector (elt pr2->target-pos 0) (elt pr2->target-pos 1) (rad2deg (car (car pr2->target-rot)))))
    (send *detect-pf* :reset-particles)

  )




;;convert r theta to xy
(defun rtheta->xy (r theta index &key (debug nil))
  (let (x y)
    (setq x (* r (cos theta)))
    (setq y (* r (sin theta)))
    (when debug
      (ros::ros-info "~d ~d x-y" x y)
      )
    (list index x y )
    )
  )

(defun distance (x y)
  (sqrt (+ (* x x) (* y y)))
  )

;;calc the distance between two lists
(defun between-distance (xy1 xy2)
  (let (x-diff y-diff)
    (setq x-diff (- (cadr xy1) (cadr xy2)))
    (setq y-diff (- (caddr xy1) (caddr xy2)))
    (distance x-diff y-diff)
    )
  )


;;return if distance is within right range or not
(defun check-likely-target (sample-length target-length &optional (error-thred *error-thred*))
  (if (and (<= sample-length (+ target-length error-thred)) (>= sample-length (- target-length error-thred))) 
      t
    nil
    )
  )

(defun distance-from-origin (xy &key (x-index 0) (y-index 1))
  (let ((x (elt xy x-index))( y (elt xy y-index)))
    (distance x y)
    )
  )



;;CALL-BACK this will be called when subscriver is activated
(defun object-detection-cb (laser-scan-msg)
  (let ((index 0) (trans nil) (angle (send laser-scan-msg :angle_min)) (angle-step (send laser-scan-msg :angle_increment)) (ranges-vec (send laser-scan-msg :ranges)) (x 0) (y 0) (xy-list nil) (object-width-likely nil) (object-height-likely nil))

    (setq ranges-vec (send laser-scan-msg :ranges))
    (ros::ros-info "------------callback is called!!----------------" )
    (setq ranges (coerce ranges-vec cons))

    ;; First Calc the x y and put those in xy-list
    ;; and add index too.
    (dolist (range ranges)
      ;;here if range > 0 it is meaningful
      (if (> range 0)
		  (push (rtheta->xy range angle index) xy-list)
		)
      (setq angle (+ angle angle-step))
      (inc index)
      )

    ;; Next estimate best combination
    (setq end-index (- (length xy-list) 1))
    (do ((i 0 (1+ i)))
		((> i end-index) (return 0))
      (do ((j (+ i 1) (1+ j)))
		  ((> j end-index) (return 0))
		(setq xy1 (elt xy-list i)     xy2 (elt xy-list j))
		;;push to array if it seems to be object-width
		(if (check-likely-target (between-distance xy1 xy2) *object-width* *error-thred*) (progn 
																						   (push (list xy1 xy2) object-width-likely)
																						   ))
		;;push to array if it seems to be object-height
		(if (check-likely-target (between-distance xy1 xy2) *object-height* *error-thred*)(progn 
																						   (push (list xy1 xy2) object-height-likely)
																						   )
          )
		)
      )

    ;;search combination of elements from each lists with id
    (setq seem-answer nil)
    (dolist (object-width-xy object-width-likely)
      (setq car-index-width (car (car object-width-xy))              cdr-index-width (car (cadr object-width-xy)))
      (dolist (object-height-xy object-height-likely)
		(setq car-index-height (car (car object-height-xy)))
		(setq cdr-index-height (car (cadr object-height-xy)))
		(if ( or (= car-index-width car-index-height) 
				 (= car-index-width cdr-index-height)
				 (= cdr-index-width car-index-height)
				 (= cdr-index-width cdr-index-height)
				 )
			(progn 
			  ;;check inner-product
			  (setq vector1 (float-vector (- (cadr (car object-width-xy)) (cadr (cadr object-width-xy))) (- (caddr (car object-width-xy)) (caddr (cadr object-width-xy)))))
			  (setq vector1 (normalize-vector vector1))
			  (setq vector2 (float-vector (- (cadr (car object-height-xy)) (cadr (cadr object-height-xy))) (- (caddr (car object-height-xy)) (caddr (cadr object-height-xy)))))
			  (setq vector2 (normalize-vector vector2))
			  (setq inner-product (v. vector1 vector2))
			  
			  (if (and ( < inner-product  *inner-threshold* ) (> inner-product (* -1 *inner-threshold*)))
				  (progn
					(push (append object-width-xy object-height-xy) seem-answer)
					)
				)
			  )
		  )
		)
      )



;;    (ros::ros-info "~A will be answers" seem-answer)
    (setq prev-xy-distance nil object-xy-params nil)
    ;;this time we use closest object-width
    (dolist (xy seem-answer)
      (setq middle-xy (list (/ (+ (cadr (car xy)) (cadr (cadr xy)) )2) (/ (+ (caddr (car xy)) (caddr (cadr xy))) 2)))
      (setq xy-distance (distance-from-origin middle-xy))

      ;;for inner-product

	  (setq former t);;this is for duplicate id's position

      (if (or (equal *object-xy* nil) (equal prev-xy-distance nil) (< xy-distance prev-xy-distance ))
		  (progn (setq *object-xy* middle-xy)
				 (setq prev-xy-distance xy-distance    object-xy-params xy    end-index 3)
				 (do ((i 0 (1+ i)))
					 ((> i 1) (return 0))
				   (do ((j 2 (1+ j)))
					   ((> j end-index) (return 0))
					 (setq id1 (car (elt xy i))     id2 (car (elt xy j)))
					 (when (equal id1 id2)
					   (if (equal j 3)
						   (setq former nil)
						 ;;(ros::ros-info "------------------------------------------joint at ~d ~d-----------------------------" id1 id2)
						 )
					   )
					 )
				   )
				 
				 ))
	  )
    (ros::ros-info "~A  answer " *object-xy*)
    ;;publish answer 
    (setq publish-list nil)
    (dolist (xy object-xy-params)
	  (setq gm
			(instance geometry_msgs::Vector3 :init
					  :x (cadr xy) :y (caddr xy) :z 0))
	  (push gm  publish-list)
	  )
	
    (setq all-input-msg (instance jsk_2013_04_pr2_610::geometry_vectors :init))
    (send all-input-msg :vectors publish-list)
    (when ( > (length publish-list) 0)
	  (ros::publish *check-topic* all-input-msg)
	  )
	)

  ;;if there are answer, continue the process
  (when (not (eq object-xy-params nil))
    ;;we calc the normal
    (setq normal-to-line (float-vector (* 1 (- (caddr (car object-xy-params)) (caddr (cadr object-xy-params)))) (* -1 (- (cadr (car object-xy-params)) (cadr (cadr object-xy-params))))))    
    ;;normalization
    ;;transform from float-vector to list
    (setq normal-to-line-list (coerce (normalize-vector normal-to-line) cons))

    ;;reset to the object center pos
    (setq *object-xy* (mapcar #'+ *object-xy* (mapcar #'(lambda (x) (* x (/ *object-height* 2))) normal-to-line-list)))

    ;;calc the rotate by using normal-to-line's y    
    (setq rot-radian (asin (elt normal-to-line 1)))

    ;;prepare target-coords
    (setq target-coords (make-coords :pos (float-vector (* 1000 (car *object-xy*)) (* 1000 (cadr *object-xy*)) -303)))
    (send target-coords :rotate rot-radian :z)

    ;;publish ObjectDetection
    (let ((res (instance posedetection_msgs::ObjectDetection :init))  
		  lst 
		  (header  (instance std_msgs::header :init
							 :stamp (send laser-scan-msg :header :stamp)
							 :frame_id *base-frame-id*)))
      (let* ((o6dp (instance posedetection_msgs::Object6DPose :init)))
	;;*base-laser-to-laser-scan*
	(send target-coords :transform *base-laser-to-laser-scan* :world)
	(send o6dp :pose (ros::coords->tf-pose target-coords))
	(send o6dp :type "object-center")
	(push o6dp lst)
        ;;
        ;; publish object_detection_marekr
        (setq marker-life 10)
        (send *target-object* :move-to (send target-coords :copy-worldcoords)
              (send *ri* :state :worldcoords *room610-origin*))
        (send *target-object* :update-descendants) ;; ???
        (send *target-object* :copy-worldcoords)   ;; ??? 
        (ros::advertise "/object_detection_marker_array" visualization_msgs::MarkerArray 5)
        (ros::publish "/object_detection_marker_array"
                      (instance visualization_msgs::MarkerArray :init
                                :markers
                                (list (text->marker-msg
                                       "object-center"
                                       (send (send *target-object* :copy-worldcoords)
                                               :translate #f(-100 0 100))
                                         (instance std_msgs::header :init
                                                   :stamp (send header :stamp)
                                                   :frame_id *room610-origin*)
                                         :ns (format nil "object_text_~A" "object-center") :color #f(1 0 0)
                                         :scale 100 :lifetime marker-life :id 0)
                                        (coords->marker-msg
                                         (send *target-object* :copy-worldcoords)
                                         (instance std_msgs::header :init
                                                   :stamp (send header :stamp)
                                                   :frame_id *room610-origin*)
                                         :ns (format nil "object_coords_~A" "object-center")
                                         ;; :size 200 :width 20
                                         :lifetime marker-life :id 1)
                                        (object->marker-msg
                                         *target-object*
                                         (instance std_msgs::header :init
                                                   :stamp (send header :stamp)
                                                   :frame_id *room610-origin*)
                                         :ns (format nil "object_body_~A" "object-center")
                                         :lifetime marker-life :id 2)
                                        )))
	)
      (send res :header header)
      (send res :objects lst)
      (ros::publish *result-topic* res)
      (ros::ros-info "PUBLISHED to ~A" *result-topic*)
      )
    )
  )


(defun detect-object ()
  (ros::subscribe "/scan_filtered_foot"
				  sensor_msgs::LaserScan
				  #'object-detection-cb)

  (init-particles)
  (ros::advertise *result-topic* posedetection_msgs::ObjectDetection)
  (ros::advertise *check-topic* jsk_2013_04_pr2_610::geometry_vectors)

  ;; get base_link to base_laser_link
  (setq *base-laser-to-laser-scan* (make-coords :pos #f(275 0 303)))

  (ros::rate 10)
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    ))

(detect-object)

