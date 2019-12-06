(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-srvs "std_srvs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :state-machine "package://roseus_smach/src/state-machine.l")

(load "package://elevator_move_base_pr2/src/utils.l")


(defclass light-button-cb-class
  :super propertied-object
  :slots (value timestamp tau))

(defmethod light-button-cb-class
  (:init ()
   (setq value 0)
   (setq timestamp (ros::time 0))
   (setq tau 1.0) ;; tau = half-life
   (ros::subscribe "light_button" std_msgs::float32 #'send self :cb))

  (:cb (msg)
   (let* ((diff (send (ros::time- (ros::time-now) timestamp) :to-sec))
          (weight (exp (* (log 0.5) (/ diff tau)))))
     (setq timestamp (ros::time-now))
     (setq value (+ (* weight value) (* (- 1 weight) (send msg :data))))))

  ;; if the last observation is too old, the score is 0
  (:value ()
   (if (< tau (send (ros::time- (ros::time-now) timestamp) :to-sec))
       (setq value 0)
     value)))


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
     (when (>= found 3)
       (return))
     (ros::sleep))
    panel-moved))

(defun set-view-target (coords camera frame-id)
  (let ((msg (instance geometry_msgs::PointStamped :init))
        (uv (send camera :screen-point (send coords :worldpos))))
    ;; out of view
    (unless (and (<= 0 (elt uv 0) (send camera :width))
                 (<= 0 (elt uv 1) (send camera :height)))
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

(defun push-button (target-coords)
  (let (rayvec via-coords (arm :rarm)
        via-angle-vector org-arm-anglev org-head-anglev reset-pose
        (rarm-reset #f(-60 70 -70 -120 160 -30 180))  ;; 2nd element 80 -> 70
        (larm-reset #f(60 70 70 -120 -160 -30 180)))  ;; 2nd element 80 -> 70

    (unless (and (boundp '*pr2*) (boundp '*ri*))
      (ros::ros-error "*pr2* or *ri* is not bound.")
      (return-from push-button nil))

    ;; check push target simply from current tuckarm-pose
    (ros::spin-once)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    ;; if the free-arm is grasping something // TODO: smart arm change motion
    (ros::ros-info (format nil "grasp -> ~A" (send *ri* :start-grasp :arms)))
    (when (< 5 (send *ri* :start-grasp :larm))
      (pr2-tuckarm-pose :rarm))
    (when (< 5 (send *ri* :start-grasp :rarm))
      (pr2-tuckarm-pose :larm))

    (setq arm (check-tuckarm-pose))
    (setq rayvec
          (normalize-vector
           (v- (float-vector
                (elt (send *pr2* :laser_tilt_mount_link_lk :worldpos) 0)
                (elt (send *pr2* :laser_tilt_mount_link_lk :worldpos) 1)
                (elt (send target-coords :worldpos) 2))
               (send target-coords :worldpos))))

    ;; via-coords
    (setq via-coords (send (send target-coords :copy-worldcoords)
                           :translate (scale 70 rayvec) :world))
    ;; push 20mm
    (setq target-coords (send (send target-coords :copy-worldcoords)
                              :translate (scale -20 rayvec) :world))

    ;; Select which arm to use
    (when arm
      (send *pr2* arm :angle-vector (case arm (:rarm rarm-reset)
                                              (:larm larm-reset)))
      (ros::ros-info (format nil "check if IK for free-arm ~A is solvable" arm))
      (unless
          (send *pr2* arm :inverse-kinematics target-coords :rotation-axis :x)
        (ros::ros-warn (format nil "could not solve IK: ~A" arm))
        (setq arm nil)))
    (unless arm
      (setq arm
            (if (plusp (elt (send (send *pr2* :transformation target-coords)
                                  :worldpos) 1))
                :larm
              :rarm)))
    (ros::ros-info (format nil "select ~A arm to push button ~A"
                           arm (send target-coords :worldpos)))

    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq org-head-anglev (send *pr2* :head :angle-vector))

    ;; reset-pose (arms only) and select arm
    (send *pr2* :reset-pose)
    (send *pr2* :head :angle-vector org-head-anglev)
    (setq reset-pose (send *pr2* :angle-vector))
    (unless (eps-v= reset-pose (send *ri* :state :potentio-vector) 2.0)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
      (send *ri* :wait-interpolation))

    (setq org-arm-anglev (send *pr2* arm :angle-vector))

    ;; start push
    (send *pr2* arm :angle-vector (case arm (:rarm rarm-reset)
                                            (:larm larm-reset)))
    (unless
        (send *pr2* arm :inverse-kinematics via-coords :rotation-axis :x)
      (ros::ros-warn
       (format nil "could not solve ik to via-coords: ~A" via-coords))
      ;; return to original pose
      (send *pr2* arm :angle-vector org-arm-anglev)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) ;; quick!
      (send *ri* :wait-interpolation)
      (return-from push-button nil))
    (setq via-angle-vector (send *pr2* :angle-vector))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
    (send *ri* :wait-interpolation)

    (unless
        (send *pr2* arm :inverse-kinematics target-coords :rotation-axis :x)
      (ros::ros-warn
       (format nil "could not solve ik to target-coords: ~A" target-coords))
      ;; return to original pose
      (send *pr2* arm :angle-vector org-arm-anglev)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) ;; quick!
      (send *ri* :wait-interpolation)
      (return-from push-button nil))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 800)
    (send *ri* :wait-interpolation)

    (send *ri* :angle-vector via-angle-vector 400)
    (send *ri* :wait-interpolation)

    ;; return original pose
    (send *pr2* arm :angle-vector org-arm-anglev)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000) ;; quick!
    t))

(defun look-button-state (userdata)
  (let ((panel-name (cdr (assoc 'panel-name userdata))) look-target look-av)
    (if (equal panel-name "/elevator_call_panel")
        (call-service
         "/narrow_stereo/left/pass_through_feature_to_call_panel/request"
         (instance std_srvs::EmptyRequest :init))
      (call-service
       "/narrow_stereo/left/pass_through_feature_to_inside_panel/request"
       (instance std_srvs::EmptyRequest :init)))

    (update-robot-position)

    ;; jsk_pr2_startup/jsk_pr2_move_base/look-forward-in-nav.l sends
    ;; angle-vector after PR2 reached goal, so we should wait for interpolation.
    (when (one-shot-subscribe "/move_base/result"
                              move_base_msgs::MoveBaseActionResult :timeout 500)
      (unix:usleep (* 200 1000))
      (send *ri* :wait-interpolation))

    (setq look-target
          (car (send *scene* :find-object
                     (format nil "~A~A" (check-current-floor *scene* *tfl*)
                             panel-name))))
    (ros::ros-info "look-at button position ~A" (send look-target :worldpos))
    (send *pr2* :reset-pose)
    (send *pr2* :head :look-at (send look-target :worldpos))
    (setq look-av (send *pr2* :angle-vector))

    ;; Move only head quickly
    (unless (eps-v= (float-vector (elt look-av 15) (elt look-av 16))
                    (float-vector (elt (send *ri* :state :potentio-vector) 15)
                                  (elt (send *ri* :state :potentio-vector) 16))
                    0.5)
      (send *ri* :angle-vector look-av 500 :head-controller)
      (send *ri* :wait-interpolation))

    ;; Move arms and torso only when it is needed (i.e. skipped inside elevator)
    (unless (eps-v= look-av (send *ri* :state :potentio-vector) 2.0)
      (send *ri* :angle-vector look-av 5000)
      (send *ri* :wait-interpolation))

    ;; Move only head quickly once more, since
    ;; jsk_pr2_startup/jsk_pr2_move_base/look-forward-in-nav.l sends
    ;; angle-vector after PR2 reached goal, so we should wait for interpolation.
    (unless (eps-v= (float-vector (elt look-av 15) (elt look-av 16))
                    (float-vector (elt (send *ri* :state :potentio-vector) 15)
                                  (elt (send *ri* :state :potentio-vector) 16))
                    0.5)
      (send *ri* :angle-vector look-av 500 :head-controller)
      (send *ri* :wait-interpolation))

    ;; Wait until latest image is available.
    (unix:usleep (* 500 1000))

    ;; nearest call panel
    (ros::ros-info "panel-name: ~A" panel-name)
    (setq *panel* (car (send *scene* :find-object panel-name)))

    ;; update
    (ros::ros-info "Looking for button...")
    (speak-jp "ボタンを探しています。")
    (ros::ros-info "Waiting for vision update...")
    (unless (update 20)
      (ros::ros-info "Could not find button.")
      (speak-jp "見つかりませんでした。")
      (return-from look-button-state nil))
    (if (equal panel-name "/elevator_call_panel")
        (call-service
         "/narrow_stereo/left/pass_through_feature_to_call_panel/stop"
         (instance std_srvs::EmptyRequest :init))
      (call-service
       "/narrow_stereo/left/pass_through_feature_to_inside_panel/stop"
       (instance std_srvs::EmptyRequest :init)))
    t))

(defun push-button-state (userdata)
  (let* ((button (cdr (assoc 'button userdata)))
         (button-coords (find-if #'(lambda (o) (string= button (send o :name)))
                                 (flatten (send *panel* :inheritance)))))
    (ros::ros-info (format nil "push button (~a) -> ~a" button button-coords))
    (speak-jp "ボタンを押します。")
    (push-button button-coords)))

(defun check-button-light-state (userdata)
  (let* ((button (cdr (assoc 'button userdata)))
         (button-coords (find-if #'(lambda (o) (string= button (send o :name)))
                                 (flatten (send *panel* :inheritance))))
         (button-state (instance light-button-cb-class :init))
         (start-tm (ros::time-now))
         (timeout 3.0))
    (call-service "/wide_stereo/left/pass_through_image_rect_color/request"
                  (instance std_srvs::EmptyRequest :init))
    (set-view-target button-coords (send *pr2* :wide_stereo-left)
                     "wide_stereo_optical_frame")
    ;; check phase
    (ros::rate 10)
    (while (and (ros::ok)
                (ros::time< (ros::time-now)
                            (ros::time+ start-tm (ros::time timeout))))
      (ros::spin-once)
      (ros::ros-info "button-state: ~A" (send button-state :value))
      (when (< 0.5 (send button-state :value))
        (ros::ros-info "Button is lighting up.")
        (speak-jp "ボタンが光りました。")
        (call-service "/wide_stereo/left/pass_through_image_rect_color/stop"
                      (instance std_srvs::EmptyRequest :init))
        (clear-costmap)
        (return-from check-button-light-state t))
      (ros::sleep))
    (ros::ros-info "Button is not lighting up.")
    (speak-jp "押せていないようです。")
    (call-service "/wide_stereo/left/pass_through_image_rect_color/stop"
                  (instance std_srvs::EmptyRequest :init))
    nil))