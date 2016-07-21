(ros::load-ros-manifest "move_base_msgs")
(load "package://jsk_maps/src/eng2-scene.l")

(unless (boundp '*scene*) (setq *scene* (make-eng2-scene)))
(unless (boundp '*tfl*) (setq *tfl* (instance ros::transform-listener :init)))

(defvar *base-frame-id* "/base_footprint")

(defvar *move-base-counter* 0)
(defun coords->movebaseactiongoal (co &optional (map "/map") (frame "/world") (stamp (ros::time-now)))
  (let ((goal (instance move_base_msgs::movebaseactiongoal :init))
        map->frame)
    (if (send *tfl* :wait-for-transform map frame (ros::time 0) 3)
        (setq map->frame (send *tfl* :lookup-transform map frame (ros::time 0)))
      (setq map->frame (make-coords)))
    (send goal :header :seq (inc *move-base-counter*))
    (send goal :header :stamp stamp)
    (send goal :header :frame_id map)
    (send goal :goal :target_pose :header :seq *move-base-counter*)
    (send goal :goal :target_pose :header :stamp stamp)
    (send goal :goal :target_pose :header :frame_id map)
    (send goal :goal :target_pose :pose
          (ros::coords->tf-pose (send (send co :copy-worldcoords) :transform map->frame :world)))
    goal))

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

(defun move-to-with-timeout (cds
                             &key
                             (move-base-action (pr2-interface-move-base-action *ri*))
                             (map "/map")
                             (frame-id "/world")
                             (timeout 300) ;; [sec]
                             (clear-costmap) ;; clear before move
                             (max-retry-count 3))
  (let ((retry-count 0)
        (start-time (ros::time-now)))
    (send move-base-action :wait-for-server)
    (when clear-costmap
      (clear-costmap))
    (send move-base-action :send-goal (coords->movebaseactiongoal cds map frame-id))
    (ros::ros-info "send-goal ~A -> ~A" (send *ri* :state :worldcoords) cds)
    (send move-base-action :spin-once)
    (while (ros::ok)
      (cond
       ((eq (send move-base-action :get-state) actionlib_msgs::GoalStatus::*SUCCEEDED*)
        (return-from move-to-with-timeout t))
       ((eq (send move-base-action :get-state) actionlib_msgs::GoalStatus::*ABORTED*)
        (cond
         ((<= (inc retry-count) max-retry-count)
          (ros::ros-warn "retrying ~A/~A (move-to ~A)" retry-count max-retry-count cds)
          (setq start-time (ros::time-now))
          (clear-costmap)
          (send move-base-action :send-goal (coords->movebaseactiongoal cds map frame-id)))
         (t (ros::ros-error "failed after retrying")
            (return))))
       ((member (send move-base-action :get-state)
                (list actionlib_msgs::GoalStatus::*PREEMPTED*
                      actionlib_msgs::GoalStatus::*RECALLED*
                      actionlib_msgs::GoalStatus::*REJECTED*))
        (ros::ros-warn "aborted")
        (return-from move-to-with-timeout nil))
       ((< timeout (send (ros::time- (ros::time-now) start-time) :to-sec))
        (ros::ros-warn "timeout cancel goal...")
        (send move-base-action :cancel-goal)
        (return-from move-to-with-timeout nil))
       (t
        (send move-base-action :spin-once)
        (ros::sleep))))
    (ros::ros-error "failure (move-to ~A)" cds)
    (return-from move-to-with-timeout nil)))

(warn ";; approach-to-target~%")
(defun approach-to-target (target &key (robot *pr2*)
                              (move-back 800.0) ;; from object [mm]
                              (relative-rotation 0.0)) ;; from object->robot [rad]
  (let ((diff-pos (send robot :difference-position target))
        (target-cds (make-cascoords))
        target-dist target-angle)
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
    (send target :assoc target-cds)
    (send target :rotate relative-rotation :z)
    (send target :dissoc target-cds)
    (ros::ros-info "target-cds: ~A worldcds: ~A" target-cds (send target-cds :worldcoords))
    (let ((cnt 2) ret
          (init-coords (send (send *ri* :state :worldcoords) :copy-worldcoords))
          (prev-coords (send (send *ri* :state :worldcoords) :copy-worldcoords)))
      (setq ret (move-to-with-timeout target-cds :frame-id *base-frame-id* :timeout 7 :max-retry-count 0))
      (while (and (not ret) (>= cnt 0))
        (send target :assoc target-cds)
        (send target-cds :replace-pos (scale 1.1 (send target-cds :pos)))
        (send target :dissoc target-cds)
        (ros::ros-info "diff prev-pos: ~A init-pos: ~A"
                       (norm (send prev-coords :difference-position (send *ri* :state :worldcoords)))
                       (norm (send init-coords :difference-position (send *ri* :state :worldcoords))))
        (when (and
               (> (norm (send init-coords :difference-position (send *ri* :state :worldcoords))) 40)
               (< (norm (send prev-coords :difference-position (send *ri* :state :worldcoords))) 40))
          (ros::ros-warn "fix direction...")
          (return-from approach-to-target t))
        (setq ret (move-to-with-timeout target-cds :frame-id *base-frame-id* :timeout 7 :max-retry-count 0))
        (setq prev-coords (send (send *ri* :state :worldcoords) :copy-worldcoords))
        (dec cnt))
      ret)
    ))

(provide :pr2-move)
