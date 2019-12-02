;; -*- coding: utf-8 -*-

(ros::roseus-add-msgs "roseus")
(ros::roseus-add-srvs "std_srvs")

(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :eng8-scene "package://jsk_maps/src/eng8-scene.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :state-machine
         "package://roseus_smach/src/state-machine.l")
(require :state-machine-actionlib
         "package://roseus_smach/src/state-machine-actionlib.l")

(load "package://elevator_move_base_pr2/src/check-elevator-open.l")
(load "package://elevator_move_base_pr2/src/database-interface.l")
(load "package://elevator_move_base_pr2/src/move-inside-elevator.l")
(load "package://elevator_move_base_pr2/src/navigation-client.l")
(load "package://elevator_move_base_pr2/src/push-elevator-button.l")
(load "package://elevator_move_base_pr2/src/ros-callback-manager.l")
(load "package://elevator_move_base_pr2/src/sanity-check.l")
(load "package://elevator_move_base_pr2/src/utils.l")


(defun initialize-env ()
  (pr2-init)
  (setq *tfl* (instance ros::transform-listener :init))
  (setq *scene* (make-scene-by-tf *tfl*))
  (setq *floors* '(("B2F" "地下2階") ("B1F" "地下1階")
                   ("1F" "1階") ("2F" "2階")
                   ("3F" "3階") ("4F" "4階")
                   ("5F" "5階") ("6F" "6階")
                   ("7F" "7階") ("8F" "8階")
                   ("9F" "9階") ("10F" "10階")
                   ("11F" "11階") ("12F" "12階"))))

(defun check-target-floor-arrived (target-floor &key (timeout 25))
  (let ((topic
         (instance ros-callback-message
                   :init roseus::StringStamped "/elevator_number/result"))
        (prev-reset-time (ros::time-now))
        (prev-floor nil)
        (cur-floor nil))
    (ros::rate 5)
    (while (ros::ok)
      (ros::spin-once)
      (when (send topic :msg)
        (setq cur-floor (string-upcase (send topic :msg :data))))
      (unless (equal cur-floor prev-floor)  ;; Elevator moved, so reset time.
        (ros::ros-info (format nil "Current floor: ~A" cur-floor))
        (setq prev-reset-time (ros::time-now))
        (setq prev-floor cur-floor))
      (when (ros::time> (ros::time-now)
                        (ros::time+ prev-reset-time (ros::time timeout)))
        (ros::warn
         (format nil "Elevator has not moved in recent ~A [sec]." timeout))
        (speak-jp "エレベータが動いていません。")
        (call-service
         "/narrow_stereo/left/pass_through_feature_to_inside_panel/stop"
         (instance std_srvs::EmptyRequest :init))
        (call-service "/narrow_stereo/left/pass_through_image_rect/stop"
                      (instance std_srvs::EmptyRequest :init))
        (return-from check-target-floor-arrived nil))
      (when (and cur-floor (substringp target-floor cur-floor))
        (call-service
         "/narrow_stereo/left/pass_through_feature_to_inside_panel/stop"
         (instance std_srvs::EmptyRequest :init))
        (call-service "/narrow_stereo/left/pass_through_image_rect/stop"
                      (instance std_srvs::EmptyRequest :init))
        (return-from check-target-floor-arrived t))
      (ros::sleep)
      )))

(defun ready-to-outside (userdata)
  (call-service
   "/narrow_stereo/left/pass_through_feature_to_inside_panel/request"
   (instance std_srvs::EmptyRequest :init))
  (call-service "/narrow_stereo/left/pass_through_image_rect/request"
                (instance std_srvs::EmptyRequest :init))
  (clear-costmap)
  (tolerance-loose)
  (send *ri* :go-pos-unsafe 0 0 90)  ;; Unsafe for faster turning left

  (update-robot-position)
  ;; look at panel
  (let ((look-target
         (or (find-if #'(lambda (x) (string= (send x :name) "/elevator_number"))
                      (send *panel* :descendants))
             *panel*)))
    (send *pr2* :head :look-at (send look-target :worldpos))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation))

  (let ((targetfloor (cdr (assoc 'target-floor userdata))))
    (unless (check-target-floor-arrived (concatenate string "/" targetfloor)
                                        :timeout 25)
      (send *ri* :go-pos-unsafe 0 0 -90)  ;; Unsafe for faster turning right
      (return-from ready-to-outside :fail))

    (dolist (floor *floors*)
      (when (equal targetfloor (elt floor 0))
        (ros::ros-info (format nil "Arrived at ~A." (elt floor 0)))
        (speak-jp (concatenate string (elt floor 1) "に着きました。"))
        (change-floor (format nil "~A/~a" (send *scene* :name)
                              (string-downcase (elt floor 0)))
                      *scene* *tfl*))))

  (unless (check-elevator-open :timeout 5)
    (send *ri* :go-pos-unsafe 0 0 -90)  ;; Unsafe for faster turning right
    (return-from ready-to-outside :fail))
  :success)

(defun take-elevator-state-machine ()
  (let ((sm (instance state-machine :init))
        (move-client (pr2-interface-move-base-action *ri*)))
    (send sm :add-node (actionlib-client-state
                        :move-front move-client :timeout 300 :retry t
                        :key 'coords->movebaseactiongoal))
    (send (send sm :node :move-front) :remap-list '((:goal . front-coords)))
    (send sm :add-node (instance state :init :push-outside (push-state-machine)
                                 :remap-list '((panel-name . outside-panel-name)
                                               (button . outside-button))))
    (send sm :add-node
          (instance state :init :speak-fail
                    '(lambda(x)
                       (ros::ros-warn "Could not take elevator.")
                       (speak-jp "エレベータに乗れませんでした。" :wait t)
                       t)))
    (send sm :add-node
          (instance state :init :move-inside (move-inside-state-machine)
                    :remap-list '((elevator-front-coords . elevator-front-coords)
                                  (inside-coords . inside-coords))))
    (send sm :add-node (instance state :init :push-inside (push-state-machine)
                                 :remap-list '((panel-name . inside-panel-name)
                                               (button . target-floor))))
    (send sm :add-node
          (instance state :init :ready-to-outside 'ready-to-outside))
    (send sm :add-node (actionlib-client-state
                        :move-outside move-client :timeout 60 :retry t
                        :key 'coords->movebaseactiongoal))
    (send (send sm :node :move-outside) :remap-list '((:goal . outside-coords)))
    (send sm :add-node (instance state :init :set-param-for-normal
                                 '(lambda (x)
                                    (pr2-tuckarm-pose)
                                    (tolerance-normal))))
    (send sm :arg-keys 'elevator-front-coords 'front-coords 'outside-panel-name
                       'outside-button 'inside-coords 'inside-panel-name
                       'target-floor 'outside-coords)
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
    sm))

(defun check-elevator-needed (userdata)
  (update-robot-position)
  (let* ((goal (cdr (assoc :goal userdata)))
         (goal-pose (ros::tf-pose-stamped->coords
                     (send goal :target_pose)))
         (cur-floor (check-current-floor *scene* *tfl*))
         (target-floor (check-current-floor *scene* *tfl* goal-pose))
         target-floor-button
         (target-coords
          (transform-pose-to-target-frame goal-pose (send *scene* :name) *tfl*))
         (up/down (cond ((send *scene* :floor< target-floor cur-floor) "down")
                        ((send *scene* :floor< cur-floor target-floor) "up")
                        (t nil))))
    (ros::ros-info
     (format nil
             "cur-floor: ~A, target-floor: ~A, up/down: ~A, target-coords: ~A"
             cur-floor target-floor up/down target-coords))
    (setq target-floor-button
          (elt (assoc target-floor *floors* :test #'string=
                      :key #'(lambda (x)
                               (format nil "~A/~A" (send *scene* :name)
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
    (set-alist 'target-floor target-floor-button userdata)
    (set-alist 'outside-button up/down userdata)
    (set-alist 'target-coords target-coords userdata)
    (pr2-tuckarm-pose)
    (not (null up/down))))

(defun elevator-move-base-state-machine ()
  (let ((sm (instance state-machine :init)))
    (send sm :add-node (instance state :init :sanity-check 'sanity-check))
    (send sm :add-node
          (instance state :init :check-elevator-needed 'check-elevator-needed))
    (send sm :add-node
          (instance state :init :speak-elevator
                    '(lambda (x)
                       (pr2-tuckarm-pose)
                       (tolerance-normal)
                       (ros::ros-info "Go to elevator.")
                       (speak-jp "エレベータに向かいます。" :wait t)
                       t)))
    (send sm :add-node
          (instance state :init :take-elevator (take-elevator-state-machine)))
    (send sm :add-node (actionlib-client-state
                        :go-to (pr2-interface-move-base-action *ri*)
                        :timeout 600 :retry t
                        :key 'coords->movebaseactiongoal))
    (send (send sm :node :go-to) :remap-list '((:goal . target-coords)))
    (send sm :arg-keys 'elevator-front-coords 'front-coords 'outside-panel-name
                       'outside-button 'inside-coords 'inside-panel-name
                       'target-floor 'outside-coords 'target-coords :goal)
    (send sm :goal-state (list :success :fail))
    (send sm :start-state :sanity-check)
    (send sm :add-transition :sanity-check :check-elevator-needed t)
    (send sm :add-transition :sanity-check :fail nil)
    (send sm :add-transition :check-elevator-needed :speak-elevator t)
    (send sm :add-transition :check-elevator-needed :go-to nil)
    (send sm :add-transition :speak-elevator :take-elevator t)
    (send sm :add-transition :take-elevator :go-to :success)
    (send sm :add-transition :take-elevator :fail :fail-inside)
    (send sm :add-transition :take-elevator :fail :fail-outside)
    (send sm :add-transition :go-to :success :succeeded)
    (send sm :add-transition :go-to :fail :failed)
    sm))
