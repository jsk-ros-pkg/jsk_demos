#!/usr/bin/env roseus


(load "package://pr2eus/pr2-interface.l")
(load "models/room610-table-object.l")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "posedetection_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "jsk_2013_04_pr2_610")
(defparameter *result-topic* "/narrow_stereo/left/ObjectDetection")
(defparameter *check-topic* "/check_topic")
(defvar *base-frame-id* "/base_footprint")

(ros::roseus "detect_table")

(setq *room610-origin* "/eng8/6f/610")
(setq *chair* (instance room610-table-object :init))
(setq *ri* (instance pr2-interface :init))
(setq *chair-width* 1.09)
(setq *chair-height* 0.63)
(setq *error-thred* 0.05) ;;0.05
(setq *chair-xy* nil)
(setq *inner-threshold* 0.2)

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

;;convert r theta to xy
(defun tmp-rtheta->xy (r theta &key (debug nil))
  (let (x y)
    (setq x (* r (cos theta)))
    (setq y (* r (sin theta)))
    (when debug
      (ros::ros-info "~d ~d x-y" x y)
      )
    (list x y )
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
(defun table-detection-cb (laser-scan-msg)
  (let ((index 0) (trans nil) (angle (send laser-scan-msg :angle_min)) (angle-step (send laser-scan-msg :angle_increment)) (ranges-vec (send laser-scan-msg :ranges)) (x 0) (y 0) (xy-list nil) (chair-width-likely nil) (chair-height-likely nil))

    (setq ranges-vec (send laser-scan-msg :ranges))
    ;;(ros::ros-info "------------callback is called!!----------------" )
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

    (setq tmp-xy-list nil)
    (dolist (range ranges)
      ;;here if range > 0 it is meaningful
      (if (> range 0)
	  (push (tmp-rtheta->xy range angle) tmp-xy-list)
	)
      (setq angle (+ angle angle-step))
      )
    ;;(pprint tmp-xy-list)


    ;; Next estimate best combination
    (setq end-index (- (length xy-list) 1))
    (do ((i 0 (1+ i)))
	((> i end-index) (return 0))
      (do ((j (+ i 1) (1+ j)))
	  ((> j end-index) (return 0))
	(setq xy1 (elt xy-list i))
	(setq xy2 (elt xy-list j))
	;;(ros::ros-info "~d distance" (between-distance xy1 xy2))
	;;push to array if it seems to be chair-width
	(if (check-likely-target (between-distance xy1 xy2) *chair-width* *error-thred*) (progn 
		(push (list xy1 xy2) chair-width-likely)
		;;(ros::ros-info "~d ~d ~d distance width" (car xy1) (car xy2) (between-distance xy1 xy2))
		
		))
	;;push to array if it seems to be chair-height
	(if (check-likely-target (between-distance xy1 xy2) *chair-height* *error-thred*)(progn 
		(push (list xy1 xy2) chair-height-likely)
		;;(ros::ros-info "~d ~d ~d distance height" (car xy1) (car xy2) (between-distance xy1 xy2))
		;;
          )
          )
	)
      )

    ;;search combination of elements from each lists with id
    (setq seem-answer nil)
    (dolist (chair-width-xy chair-width-likely)
      (setq car-index-width (car (car chair-width-xy)))
      (setq cdr-index-width (car (cadr chair-width-xy)))

      (dolist (chair-height-xy chair-height-likely)
	(setq car-index-height (car (car chair-height-xy)))
	(setq cdr-index-height (car (cadr chair-height-xy)))
	(if ( or (= car-index-width car-index-height) 
		 (= car-index-width cdr-index-height)
		 (= cdr-index-width car-index-height)
		 (= cdr-index-width cdr-index-height)
		 )
		(progn 
		;;check inner-product
		(setq vector1 (float-vector (- (cadr (car chair-width-xy)) (cadr (cadr chair-width-xy))) (- (caddr (car chair-width-xy)) (caddr (cadr chair-width-xy)))))
		(setq vector1 (normalize-vector vector1))
		(setq vector2 (float-vector (- (cadr (car chair-height-xy)) (cadr (cadr chair-height-xy))) (- (caddr (car chair-height-xy)) (caddr (cadr chair-height-xy)))))
		(setq vector2 (normalize-vector vector2))
		(setq inner-product (v. vector1 vector2))
		;;(ros::ros-info "~d inner-product before check" inner-product)
		
		(if (and ( < inner-product  *inner-threshold* ) (> inner-product (* -1 *inner-threshold*)))
			(progn
				;;(ros::ros-info "~d inner-product passed" inner-product)
	    			(push (append chair-width-xy chair-height-xy) seem-answer)
			)
			)
		)
	  )
	)
      )


             (setq width-same-index -1)
             (setq height-same-index -1)

;;    (ros::ros-info "~A will be answers" seem-answer)
    (setq prev-xy-distance nil)
    (setq chair-xy-params nil)
    ;;this time we use closest chair-width
    (dolist (xy seem-answer)
      (setq middle-xy (list (/ (+ (cadr (car xy)) (cadr (cadr xy)) )2) (/ (+ (caddr (car xy)) (caddr (cadr xy))) 2)))
      (setq xy-distance (distance-from-origin middle-xy))

      ;;for inner-product

	(setq former t);;this is for duplicate id's position

      (if (or (equal *chair-xy* nil) (equal prev-xy-distance nil) (< xy-distance prev-xy-distance ))
      (progn (setq *chair-xy* middle-xy)
	     (setq prev-xy-distance xy-distance)
	     (setq chair-xy-params xy)
	     ;;calc the same id
	     (setq end-index 3)
             
      	     (do ((i 0 (1+ i)))
        	((> i 1) (return 0))
      		(do ((j 2 (1+ j)))
        	  ((> j end-index) (return 0))
        		(setq id1 (car (elt xy i)))
	        	(setq id2 (car (elt xy j)))
			(when (equal id1 id2)
				(if (equal j 3)
					(setq former nil)
					;;(ros::ros-info "------------------------------------------joint at ~d ~d-----------------------------" id1 id2)
					)
                                ;;(ros::ros-info "------------------------------------------joint at ~d ~d-----------------------------" id1 id2)
                                (setq width-same-index id1)
                                (setq height-same-index id2)
				)
        	)
      	      )
 
	))
	)
   ;; (ros::ros-info "~A  answer ~d ~d" *chair-xy* width-same-index height-same-index)
    ;;publish answer 
    (setq publish-list nil)
    (dolist (xy chair-xy-params)
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
  (when (not (eq chair-xy-params nil))
    ;;we calc the normal
    (setq normal-to-line (float-vector (* 1 (- (caddr (car chair-xy-params)) (caddr (cadr chair-xy-params)))) (* -1 (- (cadr (car chair-xy-params)) (cadr (cadr chair-xy-params))))))
    ;;(ros::ros-info "normal-to-line ~A" normal-to-line)
    
    ;;normalization
    (setq normal-to-line (normalize-vector normal-to-line))
;;    (ros::ros-info "normal-to-line ~A" normal-to-line)

    ;;transform from float-vector to list
    (setq normal-to-line-list (coerce normal-to-line cons))

    ;;reset to the chair center pos
;;    (ros::ros-info "before chair ~A" *chair-xy*)
    
    (setq *chair-xy* (mapcar #'+ *chair-xy* (mapcar #'(lambda (x) (* x (/ *chair-height* 2))) normal-to-line-list)))
    (setq corner1 nil)
    (setq corner2 nil)
    (if (equal width-same-index (car (car chair-xy-params)))
        (setq corner1 (cadr chair-xy-params))
      (setq corner1 (car chair-xy-params))
        )
    (if (equal height-same-index (car (caddr chair-xy-params)))
        (setq corner2 (cadddr chair-xy-params))
      (setq corner2 (caddr chair-xy-params))
        )
    (setq *chair-xy* (list (/ (+ (cadr corner1) (cadr corner2) ) 2) (/ (+ (caddr corner1) (caddr corner2)) 2)))

 ;;   (ros::ros-info "chair-xy CENTER is  ~A !!!!!" *chair-xy*)
;;    (setq *chair-xy* (list (/ (+ (car chair-xy-params)) 2)))

;;    (ros::ros-info "after chair ~A" *chair-xy*)

    ;;calc the rotate by using normal-to-line's y
    
    (setq rot-radian (asin (elt normal-to-line 1)))
;;    (ros::ros-info "ros-radian ~d" rot-radian)
;;    (ros::ros-info "ros-radian y ~d" (elt normal-to-line 1))
;;    (ros::ros-info "ros-radian deg ~d" (rad2deg rot-radian))
    ;;(break)

    ;;prepare target-coords
    (setq target-coords (make-coords :pos (float-vector (* 1000 (car *chair-xy*)) (* 1000 (cadr *chair-xy*)) -303)))
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
	(send o6dp :type "table-center")
	(push o6dp lst)

        ;; publish object_detection_marekr
        (setq marker-life 10)
        (send *chair* :move-to (send target-coords :copy-worldcoords)
              (send *ri* :state :worldcoords *room610-origin*))
        (send *chair* :update-descendants) ;; ???
        (send *chair* :copy-worldcoords)   ;; ??? 
        (ros::advertise "/object_detection_marker_array" visualization_msgs::MarkerArray 5)
        (ros::publish "/object_detection_marker_array"
                      (instance visualization_msgs::MarkerArray :init
                                :markers
                                (list (text->marker-msg
                                       "table-center"
                                       (send (send *chair* :copy-worldcoords)
                                               :translate #f(-100 0 100))
                                         (instance std_msgs::header :init
                                                   :stamp (send header :stamp)
                                                   :frame_id *room610-origin*)
                                         :ns (format nil "object_text_~A" "table-center") :color #f(1 0 0)
                                         :scale 100 :lifetime marker-life :id 0)
                                        (coords->marker-msg
                                         (send *chair* :copy-worldcoords)
                                         (instance std_msgs::header :init
                                                   :stamp (send header :stamp)
                                                   :frame_id *room610-origin*)
                                         :ns (format nil "object_coords_~A" "table-center")
                                         ;; :size 200 :width 20
                                         :lifetime marker-life :id 1)
                                        (object->marker-msg
                                         *chair*
                                         (instance std_msgs::header :init
                                                   :stamp (send header :stamp)
                                                   :frame_id *room610-origin*)
                                         :ns (format nil "object_body_~A" "table-center")
                                         :lifetime marker-life :id 2)
                                        )))
	)
      (send res :header header)
      (send res :objects lst)
      (ros::publish *result-topic* res)
      ;;(ros::ros-info "PUBLISHED to ~A" *result-topic*)
      )

    )
  )


(defun detect-table ()
  (ros::subscribe "/scan_filtered_foot"
		  sensor_msgs::LaserScan
		  #'table-detection-cb)

  (ros::advertise *result-topic* posedetection_msgs::ObjectDetection)
  (ros::advertise *check-topic* jsk_2013_04_pr2_610::geometry_vectors)

  ;; get base_link to base_laser_link
  (setq *base-laser-to-laser-scan* (make-coords :pos #f(275 0 303)))

  (ros::rate 10)
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    ))

(detect-table)

