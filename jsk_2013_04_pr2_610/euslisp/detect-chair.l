#!/usr/bin/env roseus


(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "posedetection_msgs")
(defvar *result-topic* "/ObjectDetection")
(defvar *base-frame-id* "/base_footprint")

(ros::roseus "detect_chair")

(setq *chair-width* 0.38)
(setq *chair-height* 0.43)
(setq *error-thred* 0.02)
(setq *chair-xy* nil)

;;prepare tfs
(unless (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))
(unless (boundp '*tfb*)
  (setq *tfb* (instance ros::transform-broadcaster :init)))


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

(defun m->mm (xy-list)
  (let ()
    (list (* (elt xy-list 0) 1000) (* (elt xy-list 1) 1000))
    )
  )


;;CALL-BACK this will be called when subscriver is activated
(defun chair-detection-cb (laser-scan-msg)
  (let ((index 0) (trans nil) (angle (send laser-scan-msg :angle_min)) (angle-step (send laser-scan-msg :angle_increment)) (ranges-vec (send laser-scan-msg :ranges)) (x 0) (y 0) (xy-list nil) (chair-width-likely nil) (chair-height-likely nil))

    (setq ranges-vec (send laser-scan-msg :ranges))
    (ros::ros-info "callback is called!!" )
    (setq ranges (coerce ranges-vec cons))

    ;; First Calc the x y and put those in xy-list
    ;; and add index too.
    (dolist (range ranges)
      ;;(ros::ros-info "~d range ~d angle" range angle)
      ;;here if range > 0 it is meaningful
      (if (> range 0)
	  (push (rtheta->xy range angle index) xy-list)

	)
      (setq angle (+ angle angle-step))
      (inc index)
      )
    (ros::ros-info "range to xy end")    


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
	(if (check-likely-target (between-distance xy1 xy2) *chair-width* *error-thred*) (push (list xy1 xy2) chair-width-likely))
	;;push to array if it seems to be chair-height
	(if (check-likely-target (between-distance xy1 xy2) *chair-height* *error-thred*) (push (list xy1 xy2) chair-height-likely))
	)
      )
    (ros::ros-info "~d nums in xy-list-------------------------------------------------------------------------------------------------" (length xy-list))
    (ros::ros-info "~A chair-width" chair-width-likely)
    (ros::ros-info "~A chair-height" chair-height-likely)

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
	    (push (append chair-width-xy chair-height-xy) seem-answer)
	  )
	)
      )



    (ros::ros-info "~A will be answers" seem-answer)
    (setq prev-xy-distance nil)
    (setq chair-xy-params nil)
    ;;this time we use closest chair-width
    (dolist (xy seem-answer)
      (setq middle-xy (list (/ (+ (cadr (car xy)) (cadr (cadr xy)) )2) (/ (+ (caddr (car xy)) (caddr (cadr xy))) 2)))
      (setq xy-distance (distance-from-origin middle-xy))
      (if (or (= *chair-xy* nil) (= prev-xy-distance nil) (< xy-distance prev-xy-distance )) )
      (progn (setq *chair-xy* middle-xy)
	     (setq prev-xy-distance xy-distance)
	     (setq chair-xy-params xy)
	     )
      )
    (ros::ros-info "~A  answer ?" *chair-xy*)
    ;;(m->mm *chair-xy*)
    )

  ;;if there are answer, continue the process
  (when (not (eq chair-xy-params nil))
    ;;we calc the normal
    (setq normal-to-line (float-vector (* 1 (- (caddr (car chair-xy-params)) (caddr (cadr chair-xy-params)))) (* -1 (- (cadr (car chair-xy-params)) (cadr (cadr chair-xy-params))))))
    (ros::ros-info "~A  normal-line" normal-to-line)

    ;;normalization
    (setq normal-to-line (normalize-vector normal-to-line))
    (ros::ros-info "~A  normal-line" normal-to-line)

    ;;transform from float-vector to list
    (setq normal-to-line-list (coerce normal-to-line cons))
    (ros::ros-info "~A  normal-line" normal-to-line-list)

    ;;reset to the chair center pos
    (setq *chair-xy* (mapcar #'+ *chair-xy* (mapcar #'(lambda (x) (* x (/ *chair-height* 2))) normal-to-line-list)))
    (ros::ros-info "~A renew chair-xy" *chair-xy*)

    ;;calc the rotate by using normal-to-line's y
    (setq rot-radian (asin (elt normal-to-line 1)))

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
	(send o6dp :type "chair-center")
	(push o6dp lst)
	)
      (send res :header header)
      (send res :objects lst)
      (ros::publish *result-topic* res)
      )

    )
  )


(defun detect-chair ()
  (ros::subscribe "/scan_filtered2"
		  sensor_msgs::LaserScan
		  #'chair-detection-cb)

  (ros::advertise *result-topic* posedetection_msgs::ObjectDetection)

  ;; get base_link to base_laser_link
					;(unless (send *tfl* :wait-for-transform *base-frame-id* "chair-center"  (ros::time 0) 1)
					;  (ros::ros-error "could not solve ~A to ~A" *base-frame-id* "chair-center"))
  ;;(setq *base_laser_to_laser_scan* (send *tfl* :lookup-transform *base-frame-id* "base_laser_link" (ros::time 0)))
  (setq *base-laser-to-laser-scan* (make-coords :pos #f(275 0 303)))

  (ros::rate 10)
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    ))

;;;(detect-chair)

