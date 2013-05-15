#!/usr/bin/env roseus


(setq *chair-width* 0.38)
(setq *chair-height* 0.43)
(setq *error-thred* 0.02)
(setq *chair-xy* nil)

;;convert r theta to xy
(defun rtheta->xy (r theta index &key (debug nil))
 (let (x y)
 	(setq x (* r (cos theta)))
 	(setq y (* r (sin theta)))
	(when debug
         (ros::ros-info "~d ~d x_y" x y)
	)
 	(list index x y )
 )
)

(defun distance (x y)
	(sqrt (+ (* x x) (* y y)))
)

;;calc the distance between two lists
(defun between_distance (xy1 xy2)
  (let (x_diff y_diff)
  (setq x_diff (- (cadr xy1) (cadr xy2)))
  (setq y_diff (- (caddr xy1) (caddr xy2)))
  (distance x_diff y_diff)
  )
)


;;return if distance is within right range or not
(defun check-likely-target (sample-length target-length &optional (error_thred *error-thred*))
  (if (and (<= sample-length (+ target-length error_thred)) (>= sample-length (- target-length error_thred))) 
	t
	nil
	)
)

(defun distance-from-origin (xy &key (x_index 0) (y_index 1))
	(let ((x (elt xy x_index))( y (elt xy y_index)))
		(distance x y)
	)
)

(defun m->mm (xy_list)
	(let ()
	(list (* (elt xy_list 0) 1000) (* (elt xy_list 1) 1000))
	)
)


;;CALL_BACK this will be called when subscriver is activated
(defun chair-detection-cb (laserScan)
  (let ((index 0) (angle (send laserScan :angle_min)) (angle_step (send laserScan :angle_increment)) (ranges_vec (send laserScan :ranges)) (x 0) (y 0) (xy_list nil) (chair-width-likely nil) (chair-height-likely nil))

    (setq ranges_vec (send laserScan :ranges))
    (ros::ros-info "callback is called!!" )
    (setq ranges (coerce ranges_vec cons))

    ;; First Calc the x y and put those in xy_list
    ;; and add index too.
    (dolist (range ranges)
        ;;(ros::ros-info "~d range ~d angle" range angle)
	;;here if range > 0 it is meaningful
	(if (> range 0)
		(push (rtheta->xy range angle index) xy_list)

	)
	(setq angle (+ angle angle_step))
	(inc index)
	)
    (ros::ros-info "range to xy end")    


    ;; Next estimate best combination
    (setq end-index (- (length xy_list) 1))
    (do ((i 0 (1+ i)))
	((> i end-index) (return 0))
	    (do ((j (+ i 1) (1+ j)))
		((> j end-index) (return 0))
		  (setq xy1 (elt xy_list i))
		  (setq xy2 (elt xy_list j))
		  ;;(ros::ros-info "~d distance" (between_distance xy1 xy2))
		;;push to array if it seems to be chair-width
		  (if (check-likely-target (between_distance xy1 xy2) *chair-width* *error-thred*) (push (list xy1 xy2) chair-width-likely))
		;;push to array if it seems to be chair-height
		  (if (check-likely-target (between_distance xy1 xy2) *chair-height* *error-thred*) (push (list xy1 xy2) chair-height-likely))
		)
	)
    (ros::ros-info "~d nums in xy_list-------------------------------------------------------------------------------------------------" (length xy_list))
    (ros::ros-info "~A chair-width" chair-width-likely)
    (ros::ros-info "~A chair-height" chair-height-likely)

    ;;search combination of elements from each lists with id
    (setq seem_answer nil)
    (dolist (chair-width-xy chair-width-likely)
	(setq car_index_width (car (car chair-width-xy)))
	(setq cdr_index_width (car (cadr chair-width-xy)))

	(dolist (chair-height-xy chair-height-likely)
		(setq car_index_height (car (car chair-height-xy)))
		(setq cdr_index_height (car (cadr chair-height-xy)))
		(if ( or (= car_index_width car_index_height) 
			(= car_index_width cdr_index_height)
			(= cdr_index_width car_index_height)
			(= cdr_index_width cdr_index_height)
			)
		    (push (append chair-width-xy chair-height-xy) seem_answer)
		)
	)
    )
	(ros::ros-info "~A will be answers" seem_answer)
    (setq prev_xy_distance nil)
    (setq chair_xy_params nil)
    ;;this time we use closest chair-width
    (dolist (xy seem_answer)
	(setq middle_xy (list (/ (+ (cadr (car xy)) (cadr (cadr xy)) )2) (/ (+ (caddr (car xy)) (caddr (cadr xy))) 2)))
	(setq xy_distance (distance-from-origin middle_xy))
	(if (or (= *chair-xy* nil) (= prev_xy_distance nil) (< xy_distance prev_xy_distance )) )
    		(progn (setq *chair-xy* middle_xy)
			(setq prev_xy_distance xy_distance)
	 		(setq chair_xy_params xy)
		)
	)
	(ros::ros-info "~A  answer ?" *chair-xy*)
	;;(m->mm *chair-xy*)
    )

    (when (not (eq chair_xy_params nil))
    ;;we calc the normal
    (setq normal-to-line (float-vector (* 1 (- (caddr (car chair_xy_params)) (caddr (cadr chair_xy_params)))) (* -1 (- (cadr (car chair_xy_params)) (cadr (cadr chair_xy_params))))))
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
)

)


(defun detect_chair ()
(ros::roseus "detect_chair")
(ros::roseus-add-msgs "sensor_msgs")
(ros::subscribe "/scan_filtered2"
                sensor_msgs::LaserScan
                #'chair-detection-cb)

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  ))
