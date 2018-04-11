#!/usr/bin/env roseus

(ros::load-ros-manifest "elevator_move_base_pr2")

(defvar *logging* (ros::get-param "logging" nil))

(if *logging*
    (progn
      (require :attention-observation "package://jsk_demo_common/euslisp/attention-observation.l")
      (defparameter *mongo-database* "jsk_robot_lifelog")
      (defparameter *current-context* :elevator))
  (progn
    (require :pr2-interface "package://pr2eus/pr2-interface.l")
    (unless (boundp 'make-attention-action)
      (setf (symbol-function 'make-attention-action) (symbol-function 'defun)))))

;(load "package://elevator_move_base_pr2/src/add-msg-pgsql.l")
(require :database-interface "database-interface.l")
(load "package://elevator_move_base_pr2/src/navigation-client.l")
(load "package://elevator_move_base_pr2/src/push-elevator-button.l")
(load "package://elevator_move_base_pr2/src/ros-callback-manager.l")


(load "package://roseus_smach/src/state-machine-actionlib.l")

(load "package://elevator_move_base_pr2/src/move-inside-elevator.l")

(setq *node-debug* nil)
;(ros::service-call "~set_logger_level"
;                   (instance roscpp::SetLoggerLevelRequest
;                             :init :logger "ros" :level "DEBUG"))

;; これどうしよう
(setq *floors* '(("B2F" "ちかにかい") ("B1F" "ちかいっかい")
		 ("1F" "いっかい") ("2F" "にかい")
		 ("3F" "さんかい") ("4F" "よんかい")
		 ("5F" "ごかい") ("6F" "ろくかい")
		 ("7F" "ななかい") ("8F" "はちかい")
		 ("9F" "きゅうかい") ("10F" "じっかい")
		 ("11F" "じゅういっかい") ("12F" "じゅうにかい")))

(defun initialize-env ()
  (defvar *tfl* (instance ros::transform-listener :init))

  (ros::advertise "robotsound" sound_play::SoundRequest 5)
  (ros::advertise "view_target" geometry_msgs::PointStamped 1)

  ;; (setq *ri* (instance pr2-interface :init))
  ;; (setq *pr2* (pr2))
  (pr2-init)

  ;; load building scene by existing TF frame
  (unix::sleep 1) ;; wait for recieving TF messages
  (let ((frames (send *tfl* :get-frame-strings)))
    (cond ((find "eng2" frames :test #'string=)
	   (setq *scene* (make-eng2-scene)))
	  ((find "eng8" frames :test #'string=)
	   (setq *scene* (make-eng8-scene)))
	  (t
	   (ros::ros-error "There is no building frame-id in TF tree.")
	   (exit))
	  ))

  (when *node-debug*
    (if (not (boundp '*irtviewer*)) (make-irtviewer))
    (objects (list *pr2* *scene*))
    (send *irtviewer* :look-all *pr2*)
    (send *irtviewer* :draw-objects)
    (x::window-main-one)
    )
  )

(defun update-robot-position ()
  (ros::spin-once)
  (send *pr2* :move-to (send *ri* :state :worldcoords) :world)
  (send *pr2* :angle-vector (send *ri* :state))
  )

(defun update (&optional (times 100))
  (let (updated-object (count 0) (found 0) (panel-moved nil))
    ;; wait for result
    (ros::rate 10)

    (while (<= (incf count) times)
     (ros::spin-once)

     (update-robot-position)
     (setq updated-object (update-scene-by-tf *scene*))

     (when (memq *panel* updated-object)
       (setq panel-moved t)
       (incf found))
     ;;
     (print (list 'count count 'found found
		  (send *panel* :worldpos)))
     ;;
     (when *node-debug*
       (ros::spin-once)
       (send *pr2* :angle-vector (send *ri* :state))
       (send *irtviewer* :look-all *pr2*)
       (send *irtviewer* :draw-objects)
       (x::window-main-one))
     (if (>= found 5) (return))
     (ros::sleep))
    panel-moved
    ))

(defun transform-pose-to-robot-coords (coords)
  (transform-pose-to-target-frame coords "base_footprint"))

(defun set-view-target (coords camera frame-id)
  (let ((msg (instance geometry_msgs::PointStamped :init))
	(uv (send camera :screen-point (send coords :worldpos))))
    ;; out of view
    (when (not (and (<= 0 (elt uv 0) (send camera :width))
		  (<= 0 (elt uv 1) (send camera :height))))
        (ros::ros-error "screen-point: ~A is out of view (~A, ~A)"
                        uv (send camera :width) (send camera :height))
	(return-from set-view-target nil))
    (send msg :header :frame_id frame-id)
    (send msg :point :x (elt uv 0))
    (send msg :point :y (elt uv 1))
    (send msg :point :z 10) ;; radius [px]
    (ros::publish "view_target" msg)
    t))

(defclass light-button-cb-class
  :super propertied-object
  :slots (value timestamp tau))
(defmethod light-button-cb-class
  (:init ()
	 (setq value 0 timestamp (ros::time) tau 1.0) ;; tau = half-life
	 (ros::subscribe "light_button" std_msgs::float32 #'send self :cb))
  (:cb (msg)
       (let* ((diff (send (ros::time- (ros::time-now) timestamp) :to-sec))
	      (weight (exp (* (log 0.5) (/ diff tau)))))
	 (setq timestamp (ros::time-now))
	 (setq value (+ (* weight value) (* (- 1 weight) (send msg :data))))
	 ))
  ;; if the last observation is too old, the score is 0
  (:value () (if (< tau (send (ros::time- (ros::time-now) timestamp) :to-sec))
		 (setq value 0) value)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;
;; State Machine
;;

(make-attention-action look-button (userdata)
  (let ((panel-name (cdr (assoc 'panel-name userdata))) look-target)
    (send *logger* :save-robot-pose "look-button")
    ;; nearest call panel
    (ros::ros-info "panel-name: ~A" panel-name)
    (setq *panel* (car (send *scene* :find-object panel-name)))
    (setq look-target (car (send *scene* :find-object (format nil "~A~A" (check-current-floor) panel-name))))
    ;;
    (update-robot-position)
    (update-scene-by-tf *scene*)
    (when look-target
      (ros::ros-info "look-at button position ~A" (send look-target :worldpos))
      (send *pr2* :head :angle-vector #f(0 0))
      (send *pr2* :head :look-at (send look-target :worldpos))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      (send *ri* :wait-interpolation))

    (unix:sleep 2)

    ;; update
    (ros::ros-info "wait vision update")
    (unless (update 100) (return-from look-button nil))

    (when *node-debug*
      (send *irtviewer* :draw-objects))

    (send *logger* :save-obj-pose *panel* "look-button")
    ;;(insert-tf-table-by-coords *db* (ros::time-now) *scene* (send *pr2* :base_footprint) :target_id "/base_footprint")
    t ))

(make-attention-action push-button-func (userdata)
  (let* ((button (cdr (assoc 'button userdata)))
	 (button-obj
	   (find-if #'(lambda(o)(string= button (send o :name))) (flatten (send *panel* :inheritance)))))
    (ros::ros-info (format nil "push button (~a) -> ~a" button button-obj))
    (speak-jp "ぼたんをおします")
    (send *logger* :save-obj-pose button-obj "push-button")
    (push-button button-obj)
    ))

(make-attention-action check-button-light (userdata)
  (let* ((button (cdr (assoc 'button userdata)))
	 (button-coords (find-if #'(lambda(o)(string= button (send o :name)))
				(flatten (send *panel* :inheritance))))
	 (button-state (instance light-button-cb-class :init)))
    (update-robot-position)
    (set-view-target button-coords (send *pr2* :wide_stereo-left) "wide_stereo_optical_frame")
    ;; check phase
    (dotimes (i 30 (< i 30)) ;; 3 sec
      (ros::spin-once)
      (send *logger* :save-brightness (send button-state :value) "check-button")
      (when (< 0.5 (send button-state :value))
	(pprint 'button-light) (speak-jp "ぼたんがひかりました")
;;	(pr2-tuckarm-pose)
	(return))
      (unix::usleep (* 100 1000)))
    ))

(defun push-state-machine ()
  (let ((sm (instance state-machine :init)))
    (send sm :add-node (instance state :init :look 'look-button))
    (send sm :add-node (instance state :init :push 'push-button-func))
    (send sm :add-node (instance state :init :check 'check-button-light))
    (send sm :arg-keys 'button 'panel-name)
    (send sm :goal-state (list :success :fail))
    (send sm :start-state :look)
    (send sm :add-transition :look :push t)
    (send sm :add-transition :look :fail nil)
    (send sm :add-transition :push :check t)
    (send sm :add-transition :push :fail nil)
    (send sm :add-transition :check :success t)
    (send sm :add-transition :check :look nil)
    sm ))

(defun coords->movebaseactiongoal (co)
  (let ((goal (instance move_base_msgs::movebaseactiongoal :init)))
    (send goal :goal :target_pose :pose (ros::coords->tf-pose co))
    (send goal :goal :target_pose :header :frame_id (send co :name))
    goal
    ))

(defun check-until-elevator-empty (&rest args)
  (let ((start (ros::time-now)) (result nil))
    (setq *scan* nil)
    (ros::subscribe "/base_scan" sensor_msgs::LaserScan
		    #'(lambda(m)(setq *scan* m)))
    (unix::sleep 1) ;; to smart

    (while (< (send (ros::time- (ros::time-now) start) :to-sec) 30)
      (while (not *scan*) (ros::spin-once))
      ;;
      (send *tfl* :wait-for-transform "map" (send *scan* :header :frame_id)
	    (send *scan* :header :stamp) 0.5)
      (setq laser-pose
	    (send *tfl* :lookup-transform "map" (send *scan* :header :frame_id)
		  (ros::time)))
      (let ((points nil) (angle (send *scan* :angle_min)) box)
	(setq target-areas
	      (list (send (make-cube 1000 2000 1000) :translate #f(4200.0 -30700.0 0.0))
		    (send (make-cube 1000 2000 1000) :translate #f(-25800.0 -30700.0 0.0))
		    (send (make-cube 1000 2000 1000) :translate #f(34200.0 -30700.0 0.0))))
	(send-all target-areas :worldpos) ;; omajinai
	(setq points
	      (map cons
		   #'(lambda(r)
		       (prog1
			   (float-vector (* r (cos angle))
					 (* r (sin angle)) 0)
			 (incf angle (send *scan* :angle_increment))))
		   (scale 1000 (send *scan* :ranges))))
	(setq points
	      (mapcan #'(lambda(x)
			  (let ((pt (send laser-pose :transform-vector x)))
			    (when (memq :inside (send-all target-areas :insidep pt))
			      (list pt))))
		      points))
	(when (< (length points) 10)
	  (setq result t) (return))
	))
    (ros::unsubscribe "/base_scan")
    (speak-jp (if result "エレベータに入ります" "エレベータに入れません"))
    result
    ))

(defun costmap-for-elevator (userdata)
  (change-inflation-range 0.2)
  (switch-global-planner-observation nil) ;;disable tilt scan
  t)
(defun costmap-normal (&rest args)
  (change-inflation-range 0.55)
  (switch-global-planner-observation t) ;;enable tilt scan
  t)

(defun check-target-floor (userdata)
  (let ((topic (instance ros::ros-callback-message :init roseus::StringStamped "/elevator_number/result"))
	(target-floor (cdr (assoc 'target-floor userdata))))
    (dotimes (i 300 nil)
      (ros::spin-once)
      (if (and (send topic :msg)
	       (substringp (concatenate string "/" target-floor)
			   (send topic :msg :data)))
	  (return-from check-target-floor t)
        (ros::ros-info "waiting /elevator_number/result"))
      (unix::usleep (* 500 1000))
      )))

(make-attention-action ready-to-outside (userdata)
  (clear-costmap)
  (pr2-tuckarm-pose)
  (send *ri* :go-pos 0 0 90)

  (update-robot-position)
  ;; look at panel
  (let ((look-target
	 (or (find-if #'(lambda(x)(string= (send x :name) "/elevator_number"))  (send *panel* :descendants)) *panel*)))
    (send *pr2* :head :look-at (send look-target :worldpos))
    (send *pr2* :head :look-at (send look-target :worldpos)) ;; bug twice
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation))

  (unix::sleep 3) ;; this is pr2 reference coords problem -> robot 
  (check-target-floor userdata)

  (send *pr2* :head :angle-vector #f(0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)

  (let ((targetfloor (cdr (assoc 'target-floor userdata))))
    (dolist (floor *floors*)
      (when (equal targetfloor (elt floor 0))
	(speak-jp (concatenate string (elt floor 1) "につきました"))
	(change-floor (format nil "~A/~a" (send *scene* :name)
			      (string-downcase (elt floor 0))))
	)))
  (clear-costmap)
  t)

;; elevator-front-coords, front-coords, outside-panel-name, outside-button, inside-coords, inside-panel-name, target-floor, outside-coords
(defun elevator-state-machine ()
  (let ((sm (instance state-machine :init))
	(move-client (pr2-interface-move-base-action *ri*)))
    (send sm :add-node (actionlib-client-state
			:move-front move-client :timeout 300 :retry t
			:key 'coords->movebaseactiongoal))
    (send (send sm :node :move-front) :remap-list '((:goal . front-coords)))
    (send sm :add-node (instance state :init :costmap-for-elevator 'costmap-for-elevator))
    (send sm :add-node (instance state :init :push-outside (push-state-machine)
				 :remap-list '((panel-name . outside-panel-name) (button . outside-button))))
;;    (send sm :add-node (instance state :init :check-elevator-empty 'check-until-elevator-empty))
    (send sm :add-node (instance state :init :speak-fail '(lambda(x)(speak-jp "にふんたちました") t)))
    ;; (send sm :add-node (actionlib-client-state
    ;;     		:move-inside move-client :timeout 120 :retry t
    ;;     		:key 'coords->movebaseactiongoal))
    ;; (send (send sm :node :move-inside) :remap-list '((:goal . inside-coords)))

    ;; check if elevator door open (added by furushchev 2014/04/29)
    (send sm :add-node (instance state :init :move-inside (move-inside-state-machine)
                                 :remap-list '((front-coords . elevator-front-coords)
                                               (inside-coords . inside-coords))))
        (send sm :add-node (instance state :init :push-inside (push-state-machine)
				 :remap-list '((panel-name . inside-panel-name) (button . target-floor))))
    (send sm :add-node (instance state :init :ready-to-outside 'ready-to-outside))
    (send sm :add-node (actionlib-client-state
			:move-outside move-client :timeout 60 :retry t
			:key 'coords->movebaseactiongoal))
    (send (send sm :node :move-outside) :remap-list '((:goal . outside-coords)))
    (send sm :add-node (instance state :init :costmap-normal 'costmap-normal))
    (send sm :arg-keys 'elevator-front-coords 'front-coords 'outside-panel-name 'outside-button 'inside-coords
	  'inside-panel-name 'target-floor 'outside-coords :cancel)
    (send sm :goal-state (list :success :fail-outside :fail-inside))
    (send sm :start-state :move-front)
    ;;
    (send sm :add-transition :move-front :costmap-for-elevator :succeeded)
    (send sm :add-transition :move-front :fail-outside :failed)
    (send sm :add-transition :costmap-for-elevator :push-outside t)
    (send sm :add-transition :push-outside :move-inside :success)
    (send sm :add-transition :move-inside :push-inside :success)
    (send sm :add-transition :move-inside :speak-fail :fail)
    (send sm :add-transition :push-outside :push-outside :fail)
    (send sm :add-transition :speak-fail :move-front t)
    (send sm :add-transition :push-inside :ready-to-outside :success)
    (send sm :add-transition :push-inside :push-inside :fail)
    (send sm :add-transition :ready-to-outside :move-outside :success)
    (send sm :add-transition :move-outside :costmap-normal :succeeded)
    (send sm :add-transition :move-outside :fail-inside :failed)
    (send sm :add-transition :costmap-normal :success t)
    sm ))

;; elevator_move_base action server
;; target_pose :frame_id = "/map" -> convert pose to each floor local
(defun elevator-smach-initial (userdata) ;; goal -> args
  (setq *logger* (instance mongo-logger :init "elevator"))
  (update-robot-position)
  (let* ((goal (cdr (assoc :goal userdata)))
	 (goal-pose (ros::tf-pose-stamped->coords
		     (send goal :target_pose)))
	 (cur-floor (check-current-floor))
	 (target-floor (check-current-floor goal-pose)) target-floor-button
	 (target-coords (transform-pose-to-target-frame goal-pose (send *scene* :name)))
	 (up/down (cond ((send *scene* :floor< target-floor cur-floor) "down")
			((send *scene* :floor< cur-floor target-floor) "up")
			(t nil))))
    (print (list cur-floor '-> target-floor up/down target-coords))
    (setq target-floor-button
	  (elt (assoc target-floor *floors* :test #'string=
		      :key #'(lambda(x)(format nil "~A/~A" (send *scene* :name)
					       (string-downcase (elt x 0))))) 0)) ;; /eng8/1f -> 1F
    (set-alist 'inside-panel-name "/elevator_inside_panel" userdata)
    (set-alist 'outside-panel-name "/elevator_call_panel" userdata)
    ;; transform to scene frame
    (set-alist 'elevator-front-coords
               (let ((coords (send *scene* :transformation (car (send *scene* :find-object (format nil "~A/elevator-outside" cur-floor))))))
                 (send coords :name (send *scene* :name))
                 (send coords :rotate pi :z)
                 coords) userdata)
    (set-alist 'front-coords (let ((coords (send *scene* :transformation (car (send *scene* :find-object (format nil "~A/elevator_call_panel-front" cur-floor)))))) (send coords :name (send *scene* :name)) coords) userdata)
    (set-alist 'inside-coords (let ((coords (send *scene* :transformation (car (send *scene* :find-object (format nil "~A/elevator_inside_panel-front" cur-floor)))))) (send coords :name (send *scene* :name)) coords) userdata)
    (set-alist 'outside-coords (let ((coords (send *scene* :transformation (car (send *scene* :find-object (format nil "~A/elevator-outside" target-floor)))))) (send coords :name (send *scene* :name)) coords) userdata)
    (set-alist 'target-floor target-floor-button userdata) ;; only for elevator
    (set-alist 'outside-button up/down userdata)
    (set-alist 'target-coords target-coords userdata)
    (print (list cur-floor '-> target-floor up/down target-coords))
    (pr2-tuckarm-pose) ;; temporary, safety pose
    (not up/down)))

(defun elevator-smach ()
  (let ((sm (instance state-machine :init)))
    (send sm :add-node (instance state :init :initial 'elevator-smach-initial))
    (send sm :add-node (instance state :init :speak-elevator '(lambda (x) (costmap-normal)  (speak-jp "えれべーたにのります") t)))
    (send sm :add-node (instance state :init :take-elevator (elevator-state-machine)))
    (send sm :add-node (actionlib-client-state
			:go-to (pr2-interface-move-base-action *ri*)
			:timeout 600 :retry t
			:key 'coords->movebaseactiongoal))
    (send (send sm :node :go-to) :remap-list '((:goal . target-coords)))
    (send sm :arg-keys 'elevator-front-coords 'front-coords 'outside-panel-name 'outside-button 'inside-coords
	  'inside-panel-name 'target-floor 'outside-coords 'target-coords :goal :cancel)
    (send sm :goal-state (list :success :fail))
    (send sm :start-state :initial)
    (send sm :add-transition :initial :go-to t)
    (send sm :add-transition :initial :speak-elevator nil)
    (send sm :add-transition :speak-elevator :take-elevator t)
    (send sm :add-transition :take-elevator :go-to :success)
    (send sm :add-transition :take-elevator :fail :fail-inside)
    (send sm :add-transition :take-elevator :fail :fail-outside)
    (send sm :add-transition :go-to :success :succeeded)
    (send sm :add-transition :go-to :fail :failed)
    sm ))

