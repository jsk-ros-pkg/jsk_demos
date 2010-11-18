#!/usr/bin/env roseus

(ros::load-ros-manifest "elevator_move_base_pr2")

(load "package://elevator_move_base_pr2/src/add-msg-pgsql.l")
(load "package://elevator_move_base_pr2/src/navigation-client.l")
(load "package://elevator_move_base_pr2/src/push-elevator-button.l")
(load "package://elevator_move_base_pr2/src/ros-callback-manager.l")

(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus/pr2-utils.l")

(load "package://roseus_smach/src/state-machine-ros.l")

(ros::roseus "elevator_move_base")

;(ros::service-call "~set_logger_level"
;                   (instance roscpp::SetLoggerLevelRequest
;                             :init :logger "ros" :level "DEBUG"))

;; これどうしよう
(setq *floors* '(("B2F" "ちかにかい" "/eng8_b2f")
		 ("B1F" "ちかいっかい" "/eng8_b1f")
		 ("1F" "いっかい" "/eng8_1f") ("2F" "にかい" "/eng8_2f")
		 ("3F" "さんかい" "/eng8_3f") ("4F" "よんかい" "/eng8_4f")
		 ("5F" "ごかい" "/eng8_5f") ("6F" "ろくかい" "/eng8_6f")
		 ("7F" "ななかい" "/eng8_7f")))

(defvar *tfl* (instance ros::transform-listener :init))

(defun init ()

  (ros::advertise "robotsound" sound_play::SoundRequest 5)
  (ros::advertise "view_target" geometry_msgs::PointStamped 1)
  (setq *ri* (instance pr2-interface :init))
  (setq *pr2* (pr2))

  (setq *scene* (make-eng8-scene))
  (setq *all-objects* (send *scene* :objects))
  (ros::ros-info "make-all-objects : names = ~A" (send-all *all-objects* :name))

  (setq *node-debug* nil)
  (when *node-debug*
    (if (not (boundp '*irtviewer*)) (make-irtviewer))
    (objects (list *pr2* *scene*))
    (send *irtviewer* :look-all *pr2*)
    (send *irtviewer* :draw-objects)
    (x::window-main-one)
    ))

(defun update-robot-position ()
  (let (mappose)
    (ros::spin-once)
    (setq mappose (send *tfl* :lookup-transform "/base_footprint" (send *scene* :name) (ros::time 0)))
    (if mappose
	(send *scene* :move-to mappose :world))
    (send *pr2* :angle-vector (send *ri* :state))
    ))

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
  (transform-pose-to-target-frame coords "/base_footprint"))

(defun set-view-target (coords camera frame-id)
  (let ((msg (instance geometry_msgs::PointStamped :init))
	(uv (send camera :screen-point (send coords :worldpos))))
    ;; out of view
    (if (not (and (<= 0 (elt uv 0) (send camera :width))
		  (<= 0 (elt uv 1) (send camera :height))))
	(return-from set-view-target nil))
    (send msg :header :frame_id frame-id)
    (send msg :point :x (elt uv 0))
    (send msg :point :y (elt uv 1))
    (send msg :point :z 20) ;; radius [px]
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

(defun speak-jp (jp-str)
  (ros::publish "robotsound"
		(instance sound_play::SoundRequest :init
			  :sound sound_play::say
			  :command sound_play::play_once
			  :arg jp-str)))

;;
;; State Machine
;;

(defun look-button (userdata)
  (let ((panel-name (cdr (assoc 'panel-name userdata))) coords-lst target)
    ;; nearest call panel
    (setq *panel*
	  (cadr (find-extream (mapcan #'(lambda(o)(mapcar #'(lambda(c) (list c o)) (send o  :get :prior-coords)))
				      (send *scene* :find-object panel-name))
			      #'(lambda(c)(norm (send (transform-pose-to-robot-coords (car c)) :pos))) #'<)))
    (setq coords-lst (mapcar #'transform-pose-to-robot-coords (send *panel* :get :prior-coords)))
    (setq target (find-extream coords-lst #'(lambda(c)(norm (send c :pos))) #'<))
    ;;
    (update-robot-position)
    (update-scene-by-tf *scene*)
    (when target
      (ros::ros-info "look-at button position ~A" (send target :worldpos))
      (send *pr2* :head :angle-vector #f(0 0))
      (send *pr2* :head :look-at (send target :worldpos)
;	    :move-target (send *pr2* :narrow_stereo_optical_frame))
	    )
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      (send *ri* :wait-interpolation))

    (unix:sleep 2)

    ;; update
    (ros::ros-info "wait vision update")
    (unless (update 100) (return-from look-button nil))

    (when *node-debug*
      (send *irtviewer* :draw-objects))

    ;;(insert-tf-table-by-coords *db* (ros::time-now) *scene* (send *pr2* :base_footprint) :target_id "/base_footprint")
    t ))

(defun push-button-func (userdata)
  (let* ((button (cdr (assoc 'button userdata)))
	 (button-obj
	   (find-if #'(lambda(o)(string= button (send o :name))) (flatten (send *panel* :inheritance)))))
    (ros::ros-info "push button")
    (speak-jp "ぼたんをおします")
    (push-button button-obj)
    ))

(defun check-button-light (userdata)
  (let* ((button (cdr (assoc 'button userdata)))
	 (button-coords (find-if #'(lambda(o)(string= button (send o :name)))
				(flatten (send *panel* :inheritance))))
	 (button-state (instance light-button-cb-class :init)))
    (update-robot-position)
    (set-view-target button-coords (send *pr2* :wide_stereo-left-camera) "wide_stereo_optical_frame")
    ;; check phase
    (dotimes (i 30 (< i 30)) ;; 3 sec
      (ros::spin-once)
      (when (< 0.5 (send button-state :value))
	(pprint 'button-light) (speak-jp "ぼたんがひかりました")
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

(defun move-to-func (userdata &key (retry 10))
  (pr2-tuckarm-pose :rarm)
  (move-to (cdr (assoc 'coords userdata)) :retry retry))

(defun costmap-for-elevator (userdata)
  (change-inflation-range 0.2)
  (switch-global-planner-observation nil)
  (initialize-costmaps)
  t)
(defun costmap-normal (userdata)
  (change-inflation-range)
  (switch-global-planner-observation t)
  (initialize-costmaps)
  t)

(defun check-target-floor (userdata)
  (let ((topic (instance ros::ros-callback-message :init elevator_move_base_pr2::StringStamped "/elevator_number/result"))
	(target-floor (cdr (assoc 'target-floor userdata))))
    (dotimes (i 100 nil)
      (ros::spin-once)
      (if (and (send topic :msg)
	       (substringp target-floor (send topic :msg :data)))
	  (return-from check-target-floor t))
      (pprint (send topic :msg :data))
      (unix::usleep (* 500 1000))
      )))

(defun ready-to-outside (userdata)
  (clear-costmap)
  (pr2-tuckarm-pose :rarm)
  (send *ri* :go-pos 0 0 90)

  (update-robot-position)
  ;; look at panel
  (let ((look-target
	 (or (find-if #'(lambda(x)(string= (send x :name) "/elevator_number"))  (send *panel* :descendants)) *panel*)))
    (send *pr2* :head :look-at (send look-target :worldpos))
    (send *pr2* :head :look-at (send look-target :worldpos)) ;; bug twice
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000))

  (check-target-floor userdata)

  (send *pr2* :head :angle-vector #f(0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)

  (let ((targetfloor (cdr (assoc 'target-floor userdata))))
    (dolist (floor *floors*)
      (when (equal targetfloor (elt floor 0))
	(speak-jp (concatenate string (elt floor 1) "につきました"))
	(change-floor (elt floor 2)))))
  (initialize-costmaps)
  t)

;; front-coords, outside-panel-name, outside-button, inside-coords, inside-panel-name, target-floor, outside-coords
(defun elevator-state-machine ()
  (let ((sm (instance state-machine :init)))
    (send sm :add-node (instance state :init :move-front 'move-to-func :remap-list '((coords . front-coords))))
    (send sm :add-node (instance state :init :costmap-for-elevator 'costmap-for-elevator))
    (send sm :add-node (instance state :init :push-outside (push-state-machine)
				 :remap-list '((panel-name . outside-panel-name) (button . outside-button))))
    (send sm :add-node (instance state :init :move-inside '(lambda(x)(move-to-func x :retry 30))
				 :remap-list '((coords . inside-coords))))
    (send sm :add-node (instance state :init :push-inside (push-state-machine)
				 :remap-list '((panel-name . inside-panel-name) (button . target-floor))))
    (send sm :add-node (instance state :init :ready-to-outside 'ready-to-outside))
    (send sm :add-node (instance state :init :move-outside 'move-to-func :remap-list '((coords . outside-coords))))
    (send sm :add-node (instance state :init :costmap-normal 'costmap-normal))
    (send sm :arg-keys 'front-coords 'outside-panel-name 'outside-button 'inside-coords
	  'inside-panel-name 'target-floor 'outside-coords)
    (send sm :goal-state (list :success :fail-outside :fail-inside))
    (send sm :start-state :move-front)
    (send sm :add-transition :move-front :costmap-for-elevator t)
    (send sm :add-transition :move-front :fail-outside nil)
    (send sm :add-transition :costmap-for-elevator :push-outside t)
    (send sm :add-transition :push-outside :move-inside :success)
    (send sm :add-transition :push-outside :push-outside :fail)
    (send sm :add-transition :move-inside :push-inside t)
    (send sm :add-transition :move-inside :move-front nil)
    (send sm :add-transition :push-inside :ready-to-outside :success)
    (send sm :add-transition :push-inside :push-inside :fail)
    (send sm :add-transition :ready-to-outside :move-outside t)
    (send sm :add-transition :move-outside :costmap-normal t)
    (send sm :add-transition :move-outside :fail-inside nil)
    (send sm :add-transition :costmap-normal :success t)
    sm ))

;; elevator_move_base action server
;; target_pose :frame_id = "/map" -> convert pose to each floor local
(defun elevator-smach-initial (userdata) ;; goal -> args
  (let* ((goal (cdr (assoc :goal userdata)))
	 (goal-pose (ros::tf-pose-stamped->coords
		     (send goal :target_pose)))
	 (cur-floor (check-current-floor))
	 (target-floor (check-current-floor goal-pose)) target-floor-button
	 (target-coords (transform-pose-to-target-frame goal-pose target-floor))
	 (up/down (if (plusp (floor- target-floor cur-floor)) "up" "down")))
    (setq target-floor-button
	  (elt (assoc target-floor *floors* :test #'string=
		      :key #'(lambda(x)(elt x 2))) 0)) ;; /eng8_1f -> 1F
    (set-alist 'inside-panel-name "/elevator_inside_panel" userdata)
    (set-alist 'outside-panel-name "/elevator_call_panel" userdata)
    (set-alist 'front-coords *eng8-elevator-panel-front* userdata)
    (set-alist 'inside-coords *eng8-elevator-inside* userdata)
    (set-alist 'outside-coords *eng8-elevator-outside* userdata)
    (set-alist 'target-floor target-floor-button userdata) ;; only for elevator
    (set-alist 'outside-button up/down userdata)
    (set-alist 'target-coords target-coords userdata)
    (print (list target-floor up/down target-coords))
    (zerop (floor- target-floor cur-floor))))

(defun elevator-smach ()
  (let ((sm (instance state-machine :init)))
    (send sm :add-node (instance state :init :initial 'elevator-smach-initial))
    (send sm :add-node (instance state :init :speak-elevator '(lambda(x)(speak-jp "えれべーたにのります") t)))
    (send sm :add-node (instance state :init :take-elevator (elevator-state-machine)))
    (send sm :add-node (instance state :init :go-to 'move-to-func :remap-list '((coords . target-coords))))
    (send sm :arg-keys 'front-coords 'outside-panel-name 'outside-button 'inside-coords
	  'inside-panel-name 'target-floor 'outside-coords 'target-coords :goal :cancel)
    (send sm :goal-state (list :success :fail))
    (send sm :start-state :initial)
    (send sm :add-transition :initial :go-to t)
    (send sm :add-transition :initial :speak-elevator nil)
    (send sm :add-transition :speak-elevator :take-elevator t)
    (send sm :add-transition :take-elevator :go-to :success)
    (send sm :add-transition :take-elevator :fail :fail-inside)
    (send sm :add-transition :take-elevator :fail :fail-outside)
    (send sm :add-transition :go-to :success t)
    (send sm :add-transition :go-to :fail nil)
    sm ))

(defun simple-goal-cb (msg)
  (let ((pub-msg (instance move_base_msgs::MoveBaseActionGoal :init)))
    (send pub-msg :header :stamp (ros::time-now))
    (send pub-msg :goal :target_pose msg)
    (clear-costmap)
    (ros::publish "/elevator_move_base/goal" pub-msg)
    ))

;;
(init)
(ros::subscribe "/elevator_move_base_simple/goal" geometry_msgs::PoseStamped
		#'simple-goal-cb)
(ros::advertise "/elevator_move_base/goal" move_base_msgs::MoveBaseActionGoal 1)
(setq sm (elevator-smach))
(setq s (instance state-machine-action-server :init "/elevator_move_base"
                  move_base_msgs::MoveBaseAction sm))
(send s :success-state '(:success))
(speak-jp "えれべ")
(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (send s :worker)
 (ros::sleep))
