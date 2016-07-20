(ros::load-ros-manifest "jsk_recognition_msgs")
(load "package://jsk_maps/src/eng2-scene.l")

(unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))

(defvar *approach-target-bounding-box-topic* "/fcn/cluster_indices_decomposer/boxes")
(defvar *approach-target-bounding-box-param* "/fcn/target_names")
(defvar *base-frame-id* "/base_footprint")

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

(defparameter *bounding-box-msg* nil)
(defun bounding-box-topic-callback (msg)
  (setq *bounding-box-msg* msg))

(defun subscribe-bounding-box-topic ()
  (ros::ros-info "subscribe ~A" *approach-target-bounding-box-topic*)
  (ros::subscribe *approach-target-bounding-box-topic* jsk_recognition_msgs::BoundingBoxArray
                  #'bounding-box-topic-callback))

(defun unsubscribe-bounding-box-topic ()
  (ros::ros-info "unsubscribe ~A" *approach-target-bounding-box-topic*)
  (ros::unsubscribe *approach-target-bounding-box-topic*))

(defun ros-vector->list (vmsg)
  (list 
   (* 1000.0 (send vmsg :x))
   (* 1000.0 (send vmsg :y))
   (* 1000.0 (send vmsg :z))))

(defun bounding-box-msg->cube (msg)
  (unless (boundp '*tfl*)
    (setq *tfl* (instance ros::transform-listener :init)))
(labels ((ros-vector->list (vmsg)
          (list 
           (* 1000.0 (send vmsg :x))
           (* 1000.0 (send vmsg :y))
           (* 1000.0 (send vmsg :z)))))
  (let ((cube (apply #'make-cube (ros-vector->list (send msg :dimensions))))
        (robot->cube (send *tfl* :lookup-transform
                           *base-frame-id* (send msg :header :frame_id) (ros::time))))
    (send cube :transform robot->cube)
    (send cube :transform (ros::tf-pose->coords (send msg :pose)))
    cube)))

(defun approach-to (name &key (robot *pr2*)
                              (move-back 600.0) ;; from object [mm]
                              (relative-rotation 0.0)) ;; from object->robot [rad]
  (let ((target-names (ros::get-param *approach-target-bounding-box-param*))
        (cnt 0)
        label-id target-box target-angle target-dist)
    (dotimes (i (length target-names))
      (when (substringp name (elt target-names i))
        (setq label-id (1- i))
        (ros::ros-info "target object: ~A (label: ~A)" (elt target-names i) i)
        (return)))
    (unless label-id
      (ros::ros-error "target name ~A is not registered (~A)" name target-names)
      (return-from approach-to nil))
    (unwind-protect
        (progn
          (subscribe-bounding-box-topic)
          (ros::rate 3)
          (while (and (ros::ok) (null *bounding-box-msg*) (< (inc cnt) 5))
            (ros::ros-info "waiting bounding box... ~A" cnt)
            (ros::spin-once)
            (ros::sleep))
          (setq target-box
                (find-if #'(lambda (b)
                             (eq label-id (send b :label))) (send *bounding-box-msg* :boxes)))
          (unless target-box
            (ros::ros-error "target is not found in bounding boxes")
            (return-from approach-to nil))
          (setq target-obj (bounding-box-msg->cube target-box))
          (let ((diff-pos (send robot :difference-position target-obj))
                (target-cds (make-cascoords)))
            (setf (aref diff-pos 2) 0.0)
            (setq target-angle (atan2 (aref diff-pos 1) (aref diff-pos 0)))
            (ros::ros-info "diff-pos: ~A" diff-pos)
            (setq target-dist (max 0.0 (min
                                        (* 0.8 (norm diff-pos))
                                        (- (norm diff-pos) move-back))))
            (ros::ros-info "target-angle: ~A target-dist: ~A" (rad2deg target-angle) target-dist)
            (send target-cds :rotate target-angle :z :world)
            (send target-cds :translate (float-vector (* target-dist (cos target-angle))
                                                      (* target-dist (sin target-angle))
                                                      0) :world)
            (send target-obj :assoc target-cds)
            (send target-obj :rotate relative-rotation :z)
            (send target-obj :dissoc target-cds)
            (ros::ros-info "target-cds: ~A worldcds: ~A" target-cds (send target-cds :worldcoords))
            (send *ri* :move-to target-cds :frame-id *base-frame-id*)
            ;; (send *ri* :go-pos (* 0.001 target-dist (cos target-angle))
            ;;       (* 0.001 target-dist (sin target-angle))
            ;;       (rad2deg target-angle))
            ))
      ;; finally
      (progn
        (send *ri* :go-stop)
        (unsubscribe-bounding-box-topic)
        (setq *bounding-box-msg* nil)))
))

(provide :pr2-move)
