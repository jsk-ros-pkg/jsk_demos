#!/usr/bin/env roseus

(ros::roseus "elevator_move_base")

(load "package://elevator_move_base_pr2/src/state-machine-main.l")
(load "package://elevator_move_base_pr2/src/utils.l")

;;
;; State Machine for test : push buttons
;;

;; elevator_move_base action server
(defun elevator-smach-initial (userdata) ;; goal -> args
  (update-robot-position)
  (let ((cur-floor (check-current-floor *scene* *tfl*)))
    (set-alist 'home-coords (send *ri* :state :worldcoords "/map") userdata)
    (set-alist 'outside-panel-name "/elevator_call_panel" userdata)
    (set-alist 'button-name "down" userdata)
    ;; transform to scene frame
    (set-alist 'front-coords (let ((coords (send *scene* :transformation (car (send *scene* :find-object (format nil "~A/elevator_call_panel-front" cur-floor)))))) (send coords :name (send *scene* :name)) coords) userdata)
    t )) ;; always return t

(defun elevator-smach ()
  (let ((sm (instance state-machine :init)))
    (send sm :add-node (instance state :init :initial 'elevator-smach-initial))
    (send sm :add-node (instance state :init :speak-task '(lambda (x) (speak-jp "てすとでーたをとりにいきます") t)))
    (send sm :add-node (instance state :init :push (push-state-machine)))
    (send (send sm :node :push)
	  :remap-list '((button . button-name) (panel-name . outside-panel-name)))
    (send sm :add-node (actionlib-client-state
			:go-elevator (pr2-interface-move-base-action *ri*)
			:timeout 600 :retry t
			:key 'coords->movebaseactiongoal))
    (send sm :add-node (actionlib-client-state
			:go-home (pr2-interface-move-base-action *ri*)
			:timeout 600 :retry t
			:key 'coords->movebaseactiongoal))
    (send (send sm :node :go-elevator) :remap-list '((:goal . front-coords)))
    (send (send sm :node :go-home) :remap-list '((:goal . home-coords)))
    (send sm :arg-keys 'home-coords 'front-coords
          'outside-panel-name 'button-name  :goal)
    (send sm :goal-state (list :success :fail))
    (send sm :start-state :initial)
    (send sm :add-transition :initial :speak-task t)
    (send sm :add-transition :speak-task :go-elevator t)
    (send sm :add-transition :go-elevator :push :succeeded)
    (send sm :add-transition :go-elevator :fail :failed)
    (send sm :add-transition :push :go-home :success)
    (send sm :add-transition :push :push :fail)
    (send sm :add-transition :go-home :success :succeeded)
    (send sm :add-transition :go-home :fail :failed)
    sm ))

;;

(initialize-env)
(ros::advertise "view_target" geometry_msgs::PointStamped 1)

(setq sm (elevator-smach) userdata '(nil))
(ros::ros-info "initialized gen_sample_data.l")
(warn ";; (send sm :execute userdata :step 1)")

;(exit)

#|
(progn
  (let ((insp (instance state-machine-inspector :init sm)))
    (unix::sleep 2)
    (send sm :reset-state)
    (send insp :publish-structure)))
|#
