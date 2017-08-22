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

(warn "goto-dining-table~%")
(defun base-go-to-dining-table (&key (wait t))
  (if *remotelogging*
     (progn (defvar declaretive-nearby-dining-table-task-name (start-task "http://knowrob.org/kb/knowrob.owl#DeclaretiveGoal" "CalculateGoalPose"))
     (generate-subtask-relation nearby-dining-table-task-name declaretive-nearby-dining-table-task-name))
  )
  (let ((co (send *scene* :spot "/eng2/7f/room73B2-table-side1")))
    (send co :translate (float-vector 0 0 0) :world)
    (if (send *ri* :simulation-modep)
        (setq co (send *room73b2* :spot "table-side1-spot")))
    (if *remotelogging*
        (progn (finish-task declaretive-nearby-dining-table-task-name)
        (defvar perform-nearby-dining-table-task-name (start-task "http://knowrob.org/kb/knowrob.owl#CRAMPerform" "PerformGoNearbyDiningTable"))
        (generate-subtask-relation nearby-dining-table-task-name perform-nearby-dining-table-task-name)
        (defvar go-nearby-dining-table-task-name (start-task "http://knowrob.org/kb/knowrob.owl#BaseMovement" "GoNearbyDiningTable"))
        (generate-subtask-relation perform-nearby-dining-table-task-name go-nearby-dining-table-task-name))
    )
    (cond
     ((equal wait t)
      (send *ri* :move-to co)
      )
     (t
      (send *ri* :move-to-send co)
      ))
    (if *remotelogging*
      (let (query-co-decl query-co-go query-co-perform id1 res1 id2 res2 id3 res3 
           (tpos (apply #'format nil "~A ~A ~A" (coerce (send co :pos) cons)))
           (trot (apply #'format nil "~A ~A ~A ~A" (coerce (matrix2quaternion (send co :worldrot)) cons))))
            (setq query-co-decl (concatenate string "send_prolog_assert_query('rdf_assert(\\'" declaretive-nearby-dining-table-task-name "\\', knowrob:goalLocation, literal(type(xsd:string,\\'" tpos " " trot "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-co-decl))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (setq query-co-go (concatenate string "send_prolog_assert_query('rdf_assert(\\'" go-nearby-dining-table-task-name "\\', knowrob:toLocation, literal(type(xsd:string,\\'" tpos " " trot "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id2 (send-json-prolog-query query-co-go))
            (setq res2 (read-json-next-solution id2))
            (finish-json-query id2)
            (setq query-co-perform (concatenate string "send_prolog_assert_query('rdf_assert(\\'" perform-nearby-dining-table-task-name "\\', knowrob:toLocation, literal(type(xsd:string,\\'" tpos " " trot "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id3 (send-json-prolog-query query-co-perform))
            (setq res3 (read-json-next-solution id3))
            (finish-json-query id3)
            (finish-task go-nearby-dining-table-task-name)
            (finish-task perform-nearby-dining-table-task-name)
      )
    )
    t
    )
)


(warn "goto-front-of-fridge~%")
(defun base-go-to-fridge (&key (wait t))
  (if *remotelogging*
     (progn (defvar declaretive-nearby-fridge-task-name (start-task "http://knowrob.org/kb/knowrob.owl#DeclaretiveGoal" "CalculateGoalPose"))
     (generate-subtask-relation nearby-fridge-task-name declaretive-nearby-fridge-task-name))
  )
  (let ((co (send *scene* :spot "/eng2/7f/room73B2-fridge-front")))
    (send co :translate (float-vector 0 0 0) :world)
    (if (send *ri* :simulation-modep)
        (setq co (send *room73b2* :spot "fridge-front-spot")))
    (if *remotelogging*
        (progn (finish-task declaretive-nearby-fridge-task-name)
        (defvar perform-nearby-fridge-task-name (start-task "http://knowrob.org/kb/knowrob.owl#CRAMPerform" "PerformGoNearbyFridge"))
        (generate-subtask-relation nearby-fridge-task-name perform-nearby-fridge-task-name)
        (defvar go-nearby-fridge-task-name (start-task "http://knowrob.org/kb/knowrob.owl#BaseMovement" "GoNearbyFridge"))
        (generate-subtask-relation perform-nearby-fridge-task-name go-nearby-fridge-task-name))
    )
    (cond
     ((equal wait t)
      (send *ri* :move-to co)
      )
     (t
      (send *ri* :move-to-send co)
      ))
    (if *remotelogging*
      (let (query-co-decl query-co-go query-co-perform id1 res1 id2 res2 id3 res3 
           (tpos (apply #'format nil "~A ~A ~A" (coerce (send co :pos) cons)))
           (trot (apply #'format nil "~A ~A ~A ~A" (coerce (matrix2quaternion (send co :worldrot)) cons))))
            (setq query-co-decl (concatenate string "send_prolog_assert_query('rdf_assert(\\'" declaretive-nearby-fridge-task-name "\\', knowrob:goalLocation, literal(type(xsd:string,\\'" tpos " " trot "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-co-decl))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (setq query-co-go (concatenate string "send_prolog_assert_query('rdf_assert(\\'" go-nearby-fridge-task-name "\\', knowrob:toLocation, literal(type(xsd:string,\\'" tpos " " trot "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id2 (send-json-prolog-query query-co-go))
            (setq res2 (read-json-next-solution id2))
            (finish-json-query id2)
            (setq query-co-perform (concatenate string "send_prolog_assert_query('rdf_assert(\\'" perform-nearby-fridge-task-name "\\', knowrob:toLocation, literal(type(xsd:string,\\'" tpos " " trot "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id3 (send-json-prolog-query query-co-perform))
            (setq res3 (read-json-next-solution id3))
            (finish-json-query id3)
            (finish-task go-nearby-fridge-task-name)
            (finish-task perform-nearby-fridge-task-name)
      )
    )
    t
    ))

(warn "goto-front-of-kitchen-counter~%")
(defun base-go-to-kitchen-counter (&key (wait t))
  (if *remotelogging*
     (progn (defvar declaretive-nearby-kitchen-counter-task-name (start-task "http://knowrob.org/kb/knowrob.owl#DeclaretiveGoal" "CalculateGoalPose"))
     (generate-subtask-relation nearby-kitchen-counter-task-name declaretive-nearby-kitchen-counter-task-name))
  )
  (let ((co (send *scene* :spot "/eng2/7f/room73B2-sink-front0")))
    (send co :translate (float-vector 0 0 0) :world)
    (if (send *ri* :simulation-modep)
        (setq co (send *room73b2* :spot "sink-front0-spot")))
    (if *remotelogging*
        (progn (finish-task declaretive-nearby-kitchen-counter-task-name)
        (defvar perform-nearby-kitchen-counter-task-name (start-task "http://knowrob.org/kb/knowrob.owl#CRAMPerform" "PerformGoNearbyKitchenCounter"))
        (generate-subtask-relation nearby-kitchen-counter-task-name perform-nearby-kitchen-counter-task-name)
        (defvar go-nearby-kitchen-counter-task-name (start-task "http://knowrob.org/kb/knowrob.owl#BaseMovement" "GoNearbyKitchenCounter"))
        (generate-subtask-relation perform-nearby-kitchen-counter-task-name go-nearby-kitchen-counter-task-name))
    )
    (cond
     ((equal wait t)
      (send *ri* :move-to co)
      )
     (t
      (send *ri* :move-to-send co)
      ))
    (if *remotelogging*
      (let (query-co-decl query-co-go query-co-perform id1 res1 id2 res2 id3 res3 
           (tpos (apply #'format nil "~A ~A ~A" (coerce (send co :pos) cons)))
           (trot (apply #'format nil "~A ~A ~A ~A" (coerce (matrix2quaternion (send co :worldrot)) cons))))
            (setq query-co-decl (concatenate string "send_prolog_assert_query('rdf_assert(\\'" declaretive-nearby-kitchen-counter-task-name "\\', knowrob:goalLocation, literal(type(xsd:string,\\'" tpos " " trot "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-co-decl))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (setq query-co-go (concatenate string "send_prolog_assert_query('rdf_assert(\\'" go-nearby-kitchen-counter-task-name "\\', knowrob:toLocation, literal(type(xsd:string,\\'" tpos " " trot "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id2 (send-json-prolog-query query-co-go))
            (setq res2 (read-json-next-solution id2))
            (finish-json-query id2)
            (setq query-co-perform (concatenate string "send_prolog_assert_query('rdf_assert(\\'" perform-nearby-kitchen-counter-task-name "\\', knowrob:toLocation, literal(type(xsd:string,\\'" tpos " " trot "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id3 (send-json-prolog-query query-co-perform))
            (setq res3 (read-json-next-solution id3))
            (finish-json-query id3)
            (finish-task go-nearby-kitchen-counter-task-name)
            (finish-task perform-nearby-kitchen-counter-task-name)
      )
    )
    t
    ))

(defun base-go-to-fridge-init-pose (&key (wait t) (torso-lift 130) (head-pitch 0))
  (if *remotelogging*
     (progn (defvar start-pose-task-name (start-task "http://knowrob.org/kb/knowrob.owl#AtLocation" "CalculateRobotState"))
     (generate-subtask-relation preman-fridge-task-name start-pose-task-name))
  )
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (if *remotelogging*
     (progn (fill-at-location-task start-pose-task-name (send *pr2* :angle-vector))
     (finish-task start-pose-task-name)
     (defvar perform-preman-fridge-task-name (start-task "http://knowrob.org/kb/knowrob.owl#CRAMPerform" "PerformAdjustBody"))
     (generate-subtask-relation preman-fridge-task-name perform-preman-fridge-task-name)
     (defvar head-movement-task-name (start-task "http://knowrob.org/kb/knowrob.owl#HeadMovement" "LookingTowardsFridge"))
     (generate-subtask-relation perform-preman-fridge-task-name head-movement-task-name))
  )
  (send *pr2* :head :angle-vector (float-vector 0 head-pitch))
  

  (if *remotelogging*
     (progn (defvar base-movement-task-name (start-task "http://knowrob.org/kb/knowrob.owl#BaseMovement" "AdjustTorsoHeight"))
     (generate-subtask-relation perform-preman-fridge-task-name base-movement-task-name))
  )
  (send *pr2* :torso :angle-vector (float-vector torso-lift))

  (send *ri* :angle-vector (send *pr2* :angle-vector))


  (if wait
      (send *ri* :wait-interpolation))
  (if *remotelogging*
      (let (query-av query-fv id1 res1 id2 res2)
            (setq query-av (concatenate string "send_prolog_assert_query('rdf_assert(\\'" head-movement-task-name "\\', knowrob:toLocation, literal(type(xsd:string,\\'" (string head-pitch) "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-av))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (finish-task head-movement-task-name)
            (setq query-fv (concatenate string "send_prolog_assert_query('rdf_assert(\\'" base-movement-task-name "\\', knowrob:toLocation, literal(type(xsd:string,\\'" (string torso-lift) "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id2 (send-json-prolog-query query-fv))
            (setq res2 (read-json-next-solution id2))
            (finish-json-query id2)
            (finish-task base-movement-task-name)
            (finish-task perform-preman-fridge-task-name)
      )
    )
  t
  )

(defun base-go-to-dining-table-init-pose (&key (wait t) (torso-lift 130) (head-pitch 0))
  (if *remotelogging*
     (progn (defvar table-pose-task-name (start-task "http://knowrob.org/kb/knowrob.owl#AtLocation" "CalculateRobotState"))
     (generate-subtask-relation preman-dining-table-task-name table-pose-task-name))
  )
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (if *remotelogging*
     (progn (fill-at-location-task table-pose-task-name (send *pr2* :angle-vector))
     (finish-task table-pose-task-name)
     (defvar perform-preman-dining-table-task-name (start-task "http://knowrob.org/kb/knowrob.owl#CRAMPerform" "PerformAdjustBody"))
     (generate-subtask-relation preman-dining-table-task-name perform-preman-dining-table-task-name)
     (defvar head-movement-table-task-name (start-task "http://knowrob.org/kb/knowrob.owl#HeadMovement" "LookingTowardsDiningTable"))
     (generate-subtask-relation perform-preman-dining-table-task-name head-movement-table-task-name))
  )
  (send *pr2* :head :angle-vector (float-vector 0 head-pitch))
  

  (if *remotelogging*
     (progn (defvar base-movement-table-task-name (start-task "http://knowrob.org/kb/knowrob.owl#BaseMovement" "AdjustTorsoHeight"))
     (generate-subtask-relation perform-preman-dining-table-task-name base-movement-table-task-name))
  )
  (send *pr2* :torso :angle-vector (float-vector torso-lift))

  (send *ri* :angle-vector (send *pr2* :angle-vector))


  (if wait
      (send *ri* :wait-interpolation))
  (if *remotelogging*
      (let (query-av query-fv id1 res1 id2 res2)
            (setq query-av (concatenate string "send_prolog_assert_query('rdf_assert(\\'" head-movement-table-task-name "\\', knowrob:toLocation, literal(type(xsd:string,\\'" (string head-pitch) "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-av))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (finish-task head-movement-table-task-name)
            (setq query-fv (concatenate string "send_prolog_assert_query('rdf_assert(\\'" base-movement-table-task-name "\\', knowrob:toLocation, literal(type(xsd:string,\\'" (string torso-lift) "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id2 (send-json-prolog-query query-fv))
            (setq res2 (read-json-next-solution id2))
            (finish-json-query id2)
            (finish-task base-movement-table-task-name)
            (finish-task perform-preman-dining-table-task-name)
      )
    )
  t
  )

(defun base-go-to-kitchen-counter-init-pose (&key (wait t) (torso-lift 130) (head-pitch 0))
  (if *remotelogging*
     (progn (defvar counter-pose-task-name (start-task "http://knowrob.org/kb/knowrob.owl#AtLocation" "CalculateRobotState"))
     (generate-subtask-relation preman-kitchen-counter-task-name counter-pose-task-name))
  )
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (if *remotelogging*
     (progn (fill-at-location-task counter-pose-task-name (send *pr2* :angle-vector))
     (finish-task counter-pose-task-name)
     (defvar perform-preman-kitchen-counter-task-name (start-task "http://knowrob.org/kb/knowrob.owl#CRAMPerform" "PerformAdjustBody"))
     (generate-subtask-relation preman-kitchen-counter-task-name perform-preman-kitchen-counter-task-name)
     (defvar head-movement-counter-task-name (start-task "http://knowrob.org/kb/knowrob.owl#HeadMovement" "LookingTowardsKitchenCounter"))
     (generate-subtask-relation perform-preman-kitchen-counter-task-name head-movement-counter-task-name))
  )
  (send *pr2* :head :angle-vector (float-vector 0 head-pitch))
  

  (if *remotelogging*
     (progn (defvar base-movement-counter-task-name (start-task "http://knowrob.org/kb/knowrob.owl#BaseMovement" "AdjustTorsoHeight"))
     (generate-subtask-relation perform-preman-kitchen-counter-task-name base-movement-counter-task-name))
  )
  (send *pr2* :torso :angle-vector (float-vector torso-lift))

  (send *ri* :angle-vector (send *pr2* :angle-vector))


  (if wait
      (send *ri* :wait-interpolation))
  (if *remotelogging*
      (let (query-av query-fv id1 res1 id2 res2)
            (setq query-av (concatenate string "send_prolog_assert_query('rdf_assert(\\'" head-movement-counter-task-name "\\', knowrob:toLocation, literal(type(xsd:string,\\'" (string head-pitch) "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id1 (send-json-prolog-query query-av))
            (setq res1 (read-json-next-solution id1))
            (finish-json-query id1)
            (finish-task head-movement-counter-task-name)
            (setq query-fv (concatenate string "send_prolog_assert_query('rdf_assert(\\'" base-movement-counter-task-name "\\', knowrob:toLocation, literal(type(xsd:string,\\'" (string torso-lift) "\\')), \\'LoggingGraph\\')', @(false), Result)."))
            (setq id2 (send-json-prolog-query query-fv))
            (setq res2 (read-json-next-solution id2))
            (finish-json-query id2)
            (finish-task base-movement-counter-task-name)
            (finish-task perform-preman-kitchen-counter-task-name)
      )
    )
  t
  )

(defun goto-front-of-kitchen-counter (&key (torso-lift 130) (head-pitch 0) (wait t))
  (if *remotelogging*
     (progn (defvar nearby-kitchen-counter-task-name (start-task "http://knowrob.org/kb/knowrob.owl#Navigate" "GoNextToKitchenCounter"))
     (generate-subtask-relation go-to-kitchen-counter-task-name nearby-kitchen-counter-task-name))
  )
  (base-go-to-kitchen-counter :wait wait)
  (if *remotelogging*
      (finish-task nearby-kitchen-counter-task-name)
  )

  (if *remotelogging*
     (progn (defvar preman-kitchen-counter-task-name (start-task "http://knowrob.org/kb/knowrob.owl#Navigate" "GoPreManipulationPoseKitchenCounter"))
     (generate-subtask-relation go-to-kitchen-counter-task-name preman-kitchen-counter-task-name))
  )
  (base-go-to-kitchen-counter-init-pose :torso-lift torso-lift :head-pitch head-pitch :wait wait)
  (if *remotelogging*
      (finish-task preman-kitchen-counter-task-name)
  )
  )

(defun goto-front-of-fridge (&key (torso-lift 130) (head-pitch 0) (wait t))
  (if *remotelogging*
     (progn (defvar nearby-fridge-task-name (start-task "http://knowrob.org/kb/knowrob.owl#Navigate" "GoNextToFridge"))
     (generate-subtask-relation go-to-fridge-task-name nearby-fridge-task-name))
  )
  (base-go-to-fridge :wait wait)
  (if *remotelogging*
      (finish-task nearby-fridge-task-name)
  )

  (if *remotelogging*
     (progn (defvar preman-fridge-task-name (start-task "http://knowrob.org/kb/knowrob.owl#Navigate" "GoPreManipulationPoseFridge"))
     (generate-subtask-relation go-to-fridge-task-name preman-fridge-task-name))
  )
  (base-go-to-fridge-init-pose :torso-lift torso-lift :head-pitch head-pitch :wait wait)
  (if *remotelogging*
      (finish-task preman-fridge-task-name)
  )
  )

(defun goto-front-of-dining-table (&key (torso-lift 130) (head-pitch 0) (wait t))
  (if *remotelogging*
     (progn (defvar nearby-dining-table-task-name (start-task "http://knowrob.org/kb/knowrob.owl#Navigate" "GoNextToFridge"))
     (generate-subtask-relation go-to-dining-table-task-name nearby-dining-table-task-name))
  )
  (base-go-to-dining-table :wait wait)
  (if *remotelogging*
      (finish-task nearby-dining-table-task-name)
  )

  (if *remotelogging*
     (progn (defvar preman-dining-table-task-name (start-task "http://knowrob.org/kb/knowrob.owl#Navigate" "GoPreManipulationPoseDiningTable"))
     (generate-subtask-relation go-to-dining-table-task-name preman-dining-table-task-name))
  )
  (base-go-to-fridge-init-pose :torso-lift torso-lift :head-pitch head-pitch :wait wait)
  (if *remotelogging*
      (finish-task preman-dining-table-task-name)
  )
  )

(provide :pr2-move)
