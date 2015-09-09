(load "package://jsk_maps/src/eng2-scene.l")

(unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))

(warn "move-to-spot~%")
(defun move-to-spot (spot &key (scene  *scene*)
                          (angle-vector) (time 2000) (trajectory)
                          ((:clear-costmap clear-cmap) t)
                          ((:tilt-enable with-tilt-laser-obstacle-cloud) t))
  (send *ri* :ros-wait 0.0 :spin t :spin-self t) ;; attention-check ...

  (unless with-tilt-laser-obstacle-cloud
    (use-tilt-laser-obstacle-cloud nil))

  (cond
   (angle-vector
    (send *pr2* :angle-vector angle-vector)
    (send *ri* :angle-vector (send *pr2* :angle-vector) time))
   (trajectory
    (send *pr2* :angle-vector (car (last (car trajectory))))
    (send *ri* :angle-vector-sequence
          (car trajectory) (cdr trajectory))
    ))

  (if clear-cmap (clear-costmap))
  (send *pr2* :move-to (send scene :spot spot) :world)
  (prog1 (send *ri* :move-to (send scene :spot spot))
    (unless with-tilt-laser-obstacle-cloud
      (use-tilt-laser-obstacle-cloud t)))
  )
#|
(defun rtheta->xy (rtheta)
  (let ((xy))
	(dolist (e rtheta)
	(push (list (* (elt e 0) (sin (elt e 1)))
				(* (elt e 0) (cos (elt e 1))))
		  xy))
	xy))

(defun xy->ab (xy)
  (let* ((sx (reduce #'+ (mapcar #'(lambda (x) (elt x 0)) xy)))
		(sy (reduce #'+ (mapcar #'(lambda (x) (elt x 1)) xy)))
		(sxx (reduce #'+ (mapcar #'(lambda (x) (* (elt x 0) (elt x 0))) xy)))
		(syy (reduce #'+ (mapcar #'(lambda (x) (* (elt x 1) (elt x 1))) xy)))
		(sxy (reduce #'+ (mapcar #'(lambda (x) (* (elt x 0) (elt x 1))) xy)))
		(a (/ (- (* (length xy) sxy) (* sx sy)) (- (* (length xy) sxx) (* sx sx))))
		(b (/ (- (* sxx sy) (* sxy sx)) (- (* (length xy) sxx) (* sx sx)))))
	(list a b)))
|#
(defun move-with-base-scan-cb (msg)
  (flet ((rtheta->xy (rtheta)
		   (let ((xy))
			 (dolist (e rtheta)
			   (push (list (* (elt e 0) (sin (elt e 1)))
						   (* (elt e 0) (cos (elt e 1))))
					 xy))
			 xy))
		 (xy->ab (xy)
		   (let* ((sx (reduce #'+ (mapcar #'(lambda (x) (elt x 0)) xy)))
				  (sy (reduce #'+ (mapcar #'(lambda (x) (elt x 1)) xy)))
				  (sxx (reduce #'+ (mapcar #'(lambda (x) (* (elt x 0) (elt x 0))) xy)))
				  (syy (reduce #'+ (mapcar #'(lambda (x) (* (elt x 1) (elt x 1))) xy)))
				  (sxy (reduce #'+ (mapcar #'(lambda (x) (* (elt x 0) (elt x 1))) xy)))
				  (a (/ (- (* (length xy) sxy) (* sx sy)) (- (* (length xy) sxx) (* sx sx))))
				  (b (/ (- (* sxx sy) (* sxy sx)) (- (* (length xy) sxx) (* sx sx)))))
			 (list a b))))	
	(let* ((pc (send msg :ranges))
		   (angle-min (send msg :angle_min))
		   (angle-inc (send msg :angle_increment))
		   (front-pc (subseq pc (- (/ (length pc) 2) *base-scan-point-num*) (+ (/ (length pc) 2) *base-scan-point-num*)))
		   rtheta)
	  ;;	(print (length pc))
	  ;;	(print angle-min)
	  ;;	(print angle-inc)
	  (dotimes (i (length front-pc))
		(push (list (elt front-pc i)
					(+ angle-min (* (+ (- (/ (length pc) 2) *base-scan-point-num*) i) angle-inc)))
			  rtheta)
			  (print (car rtheta))
		)
	  ;;	(format t "~%")
	  (setq *distance-center* (elt pc (/ (length pc) 2)))
	  (push (xy->ab (rtheta->xy rtheta)) *averages*)
	  ))
  )

(warn "move-with-base-scan~%")
(defun move-with-base-scan (&key (point 10) (rate 50) (n 30))
  "冷蔵庫とか平らな前面のあるものの前でその面と平行になるよう動く"
  (ros::roseus "move_with_base_scan_calc")
  (ros::roseus-add-msgs "sensor_msgs")
  (ros::roseus-add-msgs "posedetection_msgs")
  (ros::roseus-add-msgs "geometry_msgs")
  (defvar *base-scan-id* "/base_scan")
  (defparameter *averages* nil)
  (defparameter *distance-center* nil)
  (defparameter *base-scan-point-num* point)
  (ros::subscribe *base-scan-id*
				  sensor_msgs::LaserScan
				  #'move-with-base-scan-cb)

  (ros::rate rate)
  (when (ros::ok)
	(dotimes (i n)
	  (ros::spin-once)
	  (ros::sleep)
	  ))
  (ros::unsubscribe *base-scan-id*)

  (format t "~A~%" *averages*)
  (let* ((average (/ (car (reduce #'(lambda (x y) (list (+ (car x) (car y)) 0)) *averages*)) (length *averages*)))
		(diff-x (* *distance-center* (- 1.0 (cos (atan average)))))
		(diff-y (* *distance-center* (sin (atan average))))
		(diff-deg (rad2deg (atan (- average)))))
	(format t "distance: ~A~%" average)
	(format t "move to (x y deg) = (~A ~A ~A)~%" diff-x diff-y diff-deg)
	(when (or (> (abs diff-x) 0.5) (> (abs diff-y) 0.5) (> (abs diff-deg) 30))
		(format t "limit~%")
		(return-from move-with-base-scan nil))
	(send *ri* :go-pos-unsafe 0 0 diff-deg)
	(send *ri* :go-pos-unsafe diff-x diff-y)
	)
  t)


(warn "goto-front-of-fridge~%")
(defun base-go-to-fridge (&key (wait t))
  (let ((co (send *scene* :spot "/eng2/7f/room73B2-fridge-front")))
    (send co :translate (float-vector 0 0 0) :world)
    (if (send *ri* :simulation-modep)
        (setq co (send *room73b2* :spot "fridge-front-spot")))
    (cond
     ((equal wait t)
      (send *ri* :move-to co)
      )
     (t
      (send *ri* :move-to-send co)
      ))
    t
    ))

(defun base-go-to-fridge-init-pose (&key (wait t) (torso-lift 130) (head-pitch 0))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *pr2* :head :angle-vector (float-vector 0 head-pitch))
  (send *pr2* :torso :angle-vector (float-vector torso-lift))
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (if wait
      (send *ri* :wait-interpolation))
  t
  )


(defun goto-front-of-fridge (&key (torso-lift 130) (head-pitch 0) (wait t))
  (base-go-to-fridge :wait wait)
  (base-go-to-fridge-init-pose :torso-lift torso-lift :head-pitch head-pitch :wait wait)
  )

(provide :pr2-move)