;; -*- coding: utf-8 -*-

(ros::roseus-add-msgs "move_base_msgs")

(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :eng8-scene "package://jsk_maps/src/eng8-scene.l")
(require :state-machine
         "package://roseus_smach/src/state-machine.l")
(require :state-machine-actionlib
         "package://roseus_smach/src/state-machine-actionlib.l")

(defvar *logging* (ros::get-param "logging" nil))

(if *logging*
    (progn
      (require :attention-observation
               "package://jsk_demo_common/euslisp/attention-observation.l")
      (defparameter *mongo-database* "jsk_robot_lifelog")
      (defparameter *current-context* :elevator))
  (progn
    (require :pr2-interface "package://pr2eus/pr2-interface.l")
    (unless (boundp 'make-attention-action)
      (setf (symbol-function 'make-attention-action)
            (symbol-function 'defun)))))

(require :database-interface "database-interface.l")
(load "package://elevator_move_base_pr2/src/navigation-client.l")
(load "package://elevator_move_base_pr2/src/push-elevator-button.l")
(load "package://elevator_move_base_pr2/src/ros-callback-manager.l")
(load "package://elevator_move_base_pr2/src/utils.l")

(load "package://elevator_move_base_pr2/src/move-inside-elevator.l")


(setq *floors* '(("B2F" "地下2階") ("B1F" "地下1階")
                 ("1F" "1階") ("2F" "2階")
                 ("3F" "3階") ("4F" "4階")
                 ("5F" "5階") ("6F" "6階")
                 ("7F" "7階") ("8F" "8階")
                 ("9F" "9階") ("10F" "10階")
                 ("11F" "11階") ("12F" "12階")))

(defun initialize-env ()
  (defvar *tfl* (instance ros::transform-listener :init))

  (ros::advertise "robotsound" sound_play::SoundRequest 5)
  (ros::advertise "view_target" geometry_msgs::PointStamped 1)

  (pr2-init)

  ;; load building scene by existing TF frame
  (unix:sleep 1) ;; wait for recieving TF messages
  (let ((frames (send *tfl* :get-frame-strings)))
    (cond ((find "eng2" frames :test #'string=)
           (setq *scene* (make-eng2-scene)))
          ((find "eng8" frames :test #'string=)
           (setq *scene* (make-eng8-scene)))
          (t
           (ros::ros-error "There is no building frame-id in TF tree.")
           (exit))
          ))
  )

(defun update-robot-position ()
  (ros::spin-once)
  (send *pr2* :move-to (send *ri* :state :worldcoords) :world)
  (send *pr2* :angle-vector (send *ri* :state))
  )

(defun update (&optional (times 20))
  (let (updated-object (count 0) (found 0) (panel-moved nil))
    ;; wait for result
    (ros::rate 10)

    (while (<= (incf count) times)
     (ros::spin-once)

     (update-robot-position)
     (setq updated-object (update-scene-by-tf *scene* *tfl*))

     (when (memq *panel* updated-object)
       (setq panel-moved t)
       (incf found))
     (ros::ros-info
      (format nil "count: ~A, found: ~A, panel position: ~A"
              count found (send *panel* :worldpos)))
     (if (>= found 2) (return))
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
      (ros::ros-warn "screen-point: ~A is out of view (~A, ~A)"
        uv (send camera :width) (send camera :height))
      (return-from set-view-target nil))
    (send msg :header :stamp (ros::time-now))
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
  (let ((panel-name (cdr (assoc 'panel-name userdata))) look-target look-av)
    (update-robot-position)

    ;; jsk_pr2_startup/jsk_pr2_move_base/look-forward-in-nav.l sends
    ;; angle-vector after PR2 reached goal, so we should wait for interpolation.
    (when (one-shot-subscribe "/move_base/result"
                              move_base_msgs::MoveBaseActionResult :timeout 500)
      (unix:usleep (* 200 1000))
      (send *ri* :wait-interpolation))

    (setq look-target
          (car (send *scene* :find-object
                     (format nil "~A~A" (check-current-floor) panel-name))))
    (ros::ros-info "look-at button position ~A" (send look-target :worldpos))
    (send *pr2* :reset-pose)
    (send *pr2* :head :look-at (send look-target :worldpos))
    (setq look-av (send *pr2* :angle-vector))

    ;; Move only head quickly
    (unless (eps-v= (float-vector (elt look-av 15) (elt look-av 16))
                    (float-vector (elt (send *ri* :state :potentio-vector) 15)
                                  (elt (send *ri* :state :potentio-vector) 16))
                    0.2)
      (send *ri* :angle-vector look-av 500 :head-controller)
      (send *ri* :wait-interpolation))

    ;; Move arms and torso
    (unless (eps-v= look-av (send *ri* :state :potentio-vector) 2.0)
      (send *ri* :angle-vector look-av 5000)
      (send *ri* :wait-interpolation))

    ;; Wait until latest image is available.
    (unix:usleep (* 500 1000))

    (when *logging*
      (send *logger* :save-robot-pose "look-button"))
    ;; nearest call panel
    (ros::ros-info "panel-name: ~A" panel-name)
    (setq *panel* (car (send *scene* :find-object panel-name)))

    ;; update
    (ros::ros-info "Looking for button...")
    (speak-jp "ボタンを探しています。")
    (ros::ros-info "Waiting for vision update...")
    (unless (update 20)
      (ros::ros-info "Could not find button.")
      (speak-jp "見つかりませんでした。" :wait t)
      (return-from look-button nil))

    (when *logging*
      (send *logger* :save-obj-pose *panel* "look-button"))
    t))

(make-attention-action push-button-func (userdata)
  (let* ((button (cdr (assoc 'button userdata)))
         (button-obj
          (find-if #'(lambda (o) (string= button (send o :name)))
                   (flatten (send *panel* :inheritance)))))
    (ros::ros-info (format nil "push button (~a) -> ~a" button button-obj))
    (speak-jp "ボタンを押します。")
    (when *logging*
      (send *logger* :save-obj-pose button-obj "push-button"))
    (push-button button-obj)
    ))

(make-attention-action check-button-light (userdata)
  (let* ((button (cdr (assoc 'button userdata)))
         (button-coords (find-if #'(lambda(o)(string= button (send o :name)))
                                 (flatten (send *panel* :inheritance))))
         (button-state (instance light-button-cb-class :init)))
    (set-view-target button-coords (send *pr2* :wide_stereo-left)
                     "wide_stereo_optical_frame")
    ;; check phase
    (ros::rate 10)
    (dotimes (i 30 (< i 30)) ;; 3 sec
      (ros::spin-once)
      (when *logging*
        (send *logger* :save-brightness
              (send button-state :value) "check-button"))
      (ros::ros-info "button-state: ~A" (send button-state :value))
      (when (< 0.5 (send button-state :value))
        (ros::ros-info "Button is lighting up.")
        (speak-jp "ボタンが光りました。")
        (return-from check-button-light t))
      (ros::sleep))
    (ros::ros-info "Button is not lighting up.")
    (speak-jp "押せていないようです。")
    nil
    ))

(defun push-state-machine ()
  (let ((sm (instance state-machine :init))
        (move-client (pr2-interface-move-base-action *ri*)))
    (send sm :add-node (instance state :init :look 'look-button))
    (send sm :add-node (instance state :init :push 'push-button-func))
    (send sm :add-node (instance state :init :check 'check-button-light))
    (send sm :arg-keys 'button 'panel-name 'front-coords)
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
                    #'(lambda (m) (setq *scan* m)))
    (unix:sleep 1) ;; to smart

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
              (list (send (make-cube 1000 2000 1000)
                          :translate #f(4200.0 -30700.0 0.0))
                    (send (make-cube 1000 2000 1000)
                          :translate #f(-25800.0 -30700.0 0.0))
                    (send (make-cube 1000 2000 1000)
                          :translate #f(34200.0 -30700.0 0.0))))
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
                            (when (memq :inside
                                        (send-all target-areas :insidep pt))
                              (list pt))))
                      points))
        (when (< (length points) 10)
          (setq result t) (return))
        ))
    (ros::unsubscribe "/base_scan")
    (ros::ros-info (if result "Go into elvator." "Cannot go into elevator."))
    (speak-jp (if result "エレベータに入ります。" "エレベータに入れません。"))
    result
    ))

(defun tolerance-normal (&rest args)
  (pr2-tuckarm-pose)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "xy_goal_tolerance" :double 0.15)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/DWAPlannerROS" "yaw_goal_tolerance" :double 0.087) ;; 5[deg]
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/global_costmap" "footprint_padding" :double 0.05)
  (ros::set-dynamic-reconfigure-param
   "/move_base_node/local_costmap" "footprint_padding" :double 0.05)
  (switch-global-planner-observation t) ;;enable tilt scan
  t)

(defun check-target-floor-arrived (userdata)
  (let ((topic (instance ros-callback-message
                         :init roseus::StringStamped "/elevator_number/result"))
        (target-floor (cdr (assoc 'target-floor userdata))))
    (ros::rate 5)
    (dotimes (i 1500 nil)  ;; timeout: 5 min
      (ros::spin-once)
      (if (and (send topic :msg)
               (substringp (concatenate string "/" target-floor)
                           (string-upcase (send topic :msg :data))))
          (return-from check-target-floor-arrived t)
        (ros::ros-info "waiting /elevator_number/result"))
      (ros::sleep))
    (ros::ros-info "Timeout (5min) reached.")
    nil))

(make-attention-action ready-to-outside (userdata)
  (clear-costmap)
  (tolerance-loose)
  (send *ri* :go-pos-unsafe 0 0 90)  ;; Unsafe for faster turning left

  (update-robot-position)
  ;; look at panel
  (let ((look-target
         (or (find-if #'(lambda(x)(string= (send x :name) "/elevator_number"))
                      (send *panel* :descendants))
             *panel*)))
    (send *pr2* :head :look-at (send look-target :worldpos))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation))

  (check-target-floor-arrived userdata)

  (let ((targetfloor (cdr (assoc 'target-floor userdata))))
    (dolist (floor *floors*)
      (when (equal targetfloor (elt floor 0))
        (ros::ros-info (format nil "Arrived at ~A." (elt floor 0)))
        (speak-jp (concatenate string (elt floor 1) "に着きました。"))
        (change-floor (format nil "~A/~a" (send *scene* :name)
                              (string-downcase (elt floor 0))))
        )))

  (unless (check-elevator-open :timeout 5)
    (send *ri* :go-pos-unsafe 0 0 -90)  ;; Unsafe for faster turning right
    (return-from ready-to-outside :fail))
  :success)

;; elevator-front-coords, front-coords, outside-panel-name, outside-button,
;; inside-coords, inside-panel-name, target-floor, outside-coords
(defun elevator-state-machine ()
  (let ((sm (instance state-machine :init))
        (move-client (pr2-interface-move-base-action *ri*)))
    (send sm :add-node (actionlib-client-state
                        :move-front move-client :timeout 300 :retry t
                        :key 'coords->movebaseactiongoal))
    (send (send sm :node :move-front) :remap-list '((:goal . front-coords)))
    (send sm :add-node (instance state :init :push-outside (push-state-machine)
                                 :remap-list '((panel-name . outside-panel-name)
                                               (button . outside-button))))
    ;; (send sm :add-node (instance state :init :check-elevator-empty
    ;;                              'check-until-elevator-empty))
    (send sm :add-node
          (instance state :init :speak-fail
                    '(lambda(x)
                       (ros::ros-warn "Could not take elevator.")
                       (speak-jp "エレベータに乗れませんでした。" :wait t)
                       t)))

    ;; check if elevator door open (added by furushchev 2014/04/29)
    (send sm :add-node
          (instance state :init :move-inside (move-inside-state-machine)
                    :remap-list '((elevator-front-coords . elevator-front-coords)
                                  (inside-coords . inside-coords))))
    (send sm :add-node
          (instance state :init :push-inside (push-state-machine)
                    :remap-list '((panel-name . inside-panel-name)
                                  (button . target-floor))))
    (send sm :add-node
          (instance state :init :ready-to-outside 'ready-to-outside))
    (send sm :add-node (actionlib-client-state
                        :move-outside move-client :timeout 60 :retry t
                        :key 'coords->movebaseactiongoal))
    (send (send sm :node :move-outside) :remap-list '((:goal . outside-coords)))
    (send sm :add-node
          (instance state :init :set-param-for-normal 'tolerance-normal))
    (send sm :arg-keys 'elevator-front-coords 'front-coords 'outside-panel-name
                       'outside-button 'inside-coords 'inside-panel-name
                       'target-floor 'outside-coords :cancel)
    (send sm :goal-state (list :success :fail-outside :fail-inside))
    (send sm :start-state :move-front)
    (send sm :add-transition :move-front :push-outside :succeeded)
    (send sm :add-transition :move-front :fail-outside :failed)
    (send sm :add-transition :push-outside :move-inside :success)
    (send sm :add-transition :push-outside :push-outside :fail)
    (send sm :add-transition :move-inside :push-inside :success)
    (send sm :add-transition :move-inside :speak-fail :fail)
    (send sm :add-transition :speak-fail :move-front t)
    (send sm :add-transition :push-inside :ready-to-outside :success)
    (send sm :add-transition :push-inside :push-inside :fail)
    (send sm :add-transition :ready-to-outside :move-outside :success)
    (send sm :add-transition :ready-to-outside :push-inside :fail)
    (send sm :add-transition :move-outside :set-param-for-normal :succeeded)
    (send sm :add-transition :move-outside :fail-inside :failed)
    (send sm :add-transition :set-param-for-normal :success t)
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
         (target-coords
          (transform-pose-to-target-frame goal-pose (send *scene* :name)))
         (up/down (cond ((send *scene* :floor< target-floor cur-floor) "down")
                        ((send *scene* :floor< cur-floor target-floor) "up")
                        (t nil))))
    (ros::ros-info
     (format nil
             "cur-floor: ~A, target-floor: ~A, up/down: ~A, target-coords: ~A"
             cur-floor target-floor up/down target-coords))
    (setq target-floor-button
          (elt (assoc target-floor *floors* :test #'string=
                      :key #'(lambda (x) (format nil "~A/~A" (send *scene* :name)
                                                 (string-downcase (elt x 0)))))
               0)) ;; /eng8/1f -> 1F
    (set-alist 'inside-panel-name "/elevator_inside_panel" userdata)
    (set-alist 'outside-panel-name "/elevator_call_panel" userdata)
    ;; transform to scene frame
    (set-alist
     'elevator-front-coords
     (let ((coords (send *scene* :transformation
                         (car (send *scene* :find-object
                                    (format nil "~A/elevator-outside"
                                            cur-floor))))))
       (send coords :name (send *scene* :name))
       (send coords :rotate pi :z)
       coords)
     userdata)
    (set-alist
     'front-coords
     (let ((coords (send *scene* :transformation
                         (car (send *scene* :find-object
                                    (format nil "~A/elevator_call_panel-front"
                                            cur-floor))))))
       (send coords :name (send *scene* :name))
       coords)
     userdata)
    (set-alist
     'inside-coords
     (let ((coords (send *scene* :transformation
                         (car (send *scene* :find-object
                                    (format nil "~A/elevator_inside_panel-front"
                                            cur-floor))))))
       (send coords :name (send *scene* :name))
       coords)
     userdata)
    (set-alist
     'outside-coords
     (let ((coords (send *scene* :transformation
                         (car (send *scene* :find-object
                                    (format nil "~A/elevator-outside"
                                            target-floor))))))
       (send coords :name (send *scene* :name))
       coords)
     userdata)
    (set-alist 'target-floor target-floor-button userdata) ;; only for elevator
    (set-alist 'outside-button up/down userdata)
    (set-alist 'target-coords target-coords userdata)
    (pr2-tuckarm-pose) ;; temporary, safety pose
    (not up/down)))

(defun elevator-smach ()
  (let ((sm (instance state-machine :init)))
    (send sm :add-node (instance state :init :initial 'elevator-smach-initial))
    (send sm :add-node
          (instance state :init :speak-elevator
                    '(lambda (x)
                       (tolerance-normal)
                       (ros::ros-info "Go to elevator.")
                       (speak-jp "エレベータに向かいます。" :wait t)
                       t)))
    (send sm :add-node
          (instance state :init :take-elevator (elevator-state-machine)))
    (send sm :add-node (actionlib-client-state
                        :go-to (pr2-interface-move-base-action *ri*)
                        :timeout 600 :retry t
                        :key 'coords->movebaseactiongoal))
    (send (send sm :node :go-to) :remap-list '((:goal . target-coords)))
    (send sm :arg-keys 'elevator-front-coords 'front-coords 'outside-panel-name
                       'outside-button 'inside-coords 'inside-panel-name
                       'target-floor 'outside-coords 'target-coords :goal
                       :cancel)
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

