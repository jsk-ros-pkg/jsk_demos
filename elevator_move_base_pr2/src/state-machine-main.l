;; -*- coding: utf-8 -*-

(ros::roseus-add-msgs "roseus")
(ros::roseus-add-srvs "std_srvs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :state-machine
         "package://roseus_smach/src/state-machine.l")
(require :state-machine-actionlib
         "package://roseus_smach/src/state-machine-actionlib.l")

;; (load "package://elevator_move_base_pr2/src/database-interface.l")
(load "package://elevator_move_base_pr2/src/state/check-elevator-needed.l")
(load "package://elevator_move_base_pr2/src/state/check-elevator-open.l")
(load "package://elevator_move_base_pr2/src/state/push-elevator-button.l")
(load "package://elevator_move_base_pr2/src/state/ready-to-outside.l")
(load "package://elevator_move_base_pr2/src/state/sanity-check.l")
(load "package://elevator_move_base_pr2/src/state/speak-to-human-behind.l")
(load "package://elevator_move_base_pr2/src/utils.l")


(defvar *floors*)
;; (defvar *ri*)
(defvar *scene*)
(defvar *tfl*)


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

(defun push-state-machine ()
  (let ((sm (instance state-machine :init)))
    (send sm :add-node (instance state :init :look 'look-button-state))
    (send sm :add-node (instance state :init :push 'push-button-state))
    (send sm :add-node (instance state :init :check 'check-button-light-state))
    (send sm :arg-keys 'button 'panel-name 'push-arm)
    (send sm :goal-state (list :succeed))
    (send sm :start-state :look)
    (send sm :add-transition :look :push t)
    (send sm :add-transition :look :look nil)
    (send sm :add-transition :push :check t)
    (send sm :add-transition :push :look nil)
    (send sm :add-transition :check :succeed t)
    (send sm :add-transition :check :look nil)
    sm))

(defun move-inside-state-machine ()
  (let ((sm (instance state-machine :init))
        (move-client (pr2-interface-move-base-action *ri*)))
    (send sm :add-node (instance state :init :set-param-for-elevator-outside
                                 'tolerance-loose))
    (send sm :add-node
          (instance state :init :move-elevator-front
                    '(lambda (userdata)
                       (send *ri* :move-to
                             (cdr (assoc 'elevator-front-coords userdata))
                             :retry 2))))
    (send sm :add-node (instance state :init :set-param-for-elevator-inside
                                 'tolerance-strict))
    (send sm :add-node
          (instance state :init :check-elevator-open
                    '(lambda (x) (check-elevator-open :timeout 300))))
    (send sm :add-node (instance state :init :speak-to-human-behind
                                 'speak-to-human-behind))
    (send sm :add-node
          (instance state :init :move-elevator-inside
                    '(lambda (userdata)
                       (send *ri* :move-to
                             (cdr (assoc 'inside-coords userdata))
                             :retry 2))))
    (send sm :arg-keys 'elevator-front-coords 'inside-coords)
    (send sm :goal-state (list :succeed :fail))
    (send sm :start-state :set-param-for-elevator-outside)
    (send sm :add-transition
          :set-param-for-elevator-outside :move-elevator-front t)
    (send sm :add-transition
          :move-elevator-front :set-param-for-elevator-inside t)
    (send sm :add-transition :move-elevator-front :fail nil)
    (send sm :add-transition
          :set-param-for-elevator-inside :check-elevator-open t)
    (send sm :add-transition :check-elevator-open :speak-to-human-behind t)
    (send sm :add-transition :check-elevator-open :fail nil)
    (send sm :add-transition :speak-to-human-behind :move-elevator-inside t)
    (send sm :add-transition :move-elevator-inside :succeed t)
    (send sm :add-transition :move-elevator-inside :fail nil)
    sm))

(defun take-elevator-state-machine ()
  (let ((sm (instance state-machine :init))
        (move-client (pr2-interface-move-base-action *ri*)))
    (send sm :add-node
          (instance state :init :move-front
                    '(lambda (userdata)
                       (send *ri* :move-to
                             (cdr (assoc 'front-coords userdata))
                             :retry 10))))
    (send sm :add-node (instance state :init :push-outside (push-state-machine)
                                 :remap-list '((panel-name . outside-panel-name)
                                               (button . outside-button))))
    (send sm :add-node
          (instance state :init :speak-fail
                    '(lambda (x)
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
    (send sm :add-node
          (instance state :init :move-outside
                    '(lambda (userdata)
                       (unix:sleep 1)  ;; Wait for door opening
                       (clear-costmap)
                       (send *ri* :move-to
                             (cdr (assoc 'outside-coords userdata))
                             :retry 10))))
    (send sm :add-node
          (instance state :init :prepare-normal-movement
                    '(lambda (userdata)
                       ;; Free arm may hold something,
                       ;; so keep it away from base laser
                       (pr2-tuckarm-pose
                        (case (cdr (assoc 'push-arm userdata))
                          (:rarm :larm)
                          (:larm :rarm)))
                       (call-service "look_forward_in_navigation/start"
                                     (instance std_srvs::EmptyRequest :init))
                       (restore-params))))
    (send sm :arg-keys 'elevator-front-coords 'front-coords 'outside-panel-name
                       'outside-button 'inside-coords 'inside-panel-name
                       'target-floor 'outside-coords 'push-arm)
    (send sm :goal-state (list :succeed :fail-outside :fail-inside))
    (send sm :start-state :move-front)
    (send sm :add-transition :move-front :push-outside t)
    (send sm :add-transition :move-front :fail-outside nil)
    (send sm :add-transition :push-outside :move-inside :succeed)
    (send sm :add-transition :move-inside :push-inside :succeed)
    (send sm :add-transition :move-inside :speak-fail :fail)
    (send sm :add-transition :speak-fail :move-front t)
    (send sm :add-transition :push-inside :ready-to-outside :succeed)
    (send sm :add-transition :ready-to-outside :move-outside :succeed)
    (send sm :add-transition :ready-to-outside :push-inside :fail)
    (send sm :add-transition :move-outside :prepare-normal-movement t)
    (send sm :add-transition :move-outside :fail-inside nil)
    (send sm :add-transition :prepare-normal-movement :succeed t)
    sm))

(defun elevator-move-base-state-machine ()
  (let ((sm (instance state-machine :init)))
    (send sm :add-node
          (instance state :init :check-elevator-needed 'check-elevator-needed))
    (send sm :add-node (instance state :init :sanity-check 'sanity-check))
    (send sm :add-node
          (instance state :init :speak-elevator
                    '(lambda (userdata)
                       ;; Free arm may hold something,
                       ;; so keep it away from base laser
                       (pr2-tuckarm-pose
                        (case (cdr (assoc 'push-arm userdata))
                          (:rarm :larm)
                          (:larm :rarm)))
                       (store-params)
                       (call-service "look_forward_in_navigation/stop"
                                     (instance std_srvs::EmptyRequest :init))
                       (tolerance-medium)
                       (ros::ros-info "Go to elevator.")
                       (speak-jp "エレベータに向かいます。" :wait t)
                       t)))
    (send sm :add-node
          (instance state :init :take-elevator (take-elevator-state-machine)))
    (send sm :add-node (actionlib-client-state
                        :go-to (pr2-interface-move-base-action *ri*)
                        :timeout 300 :retry t
                        :key 'coords->movebaseactiongoal))
    (send (send sm :node :go-to) :remap-list '((:goal . target-coords)))
    (send sm :arg-keys 'elevator-front-coords 'front-coords 'outside-panel-name
                       'outside-button 'inside-coords 'inside-panel-name
                       'target-floor 'outside-coords 'target-coords
                       'push-arm :goal)
    (send sm :goal-state (list :succeed :fail))
    (send sm :start-state :check-elevator-needed)
    (send sm :add-transition :check-elevator-needed :sanity-check t)
    (send sm :add-transition :check-elevator-needed :go-to nil)
    (send sm :add-transition :sanity-check :speak-elevator t)
    (send sm :add-transition :sanity-check :fail nil)
    (send sm :add-transition :speak-elevator :take-elevator t)
    (send sm :add-transition :take-elevator :go-to :succeed)
    (send sm :add-transition :take-elevator :fail :fail-inside)
    (send sm :add-transition :take-elevator :fail :fail-outside)
    (send sm :add-transition :go-to :succeed :succeeded)
    (send sm :add-transition :go-to :fail :failed)
    sm))
