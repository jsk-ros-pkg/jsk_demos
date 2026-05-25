(ros::roseus "action_server")
(ros::load-ros-package "fridge_demo_bt")
(load "package://roseus_bt/euslisp/nodes.l")

(require :main "package://detect_cans_in_fridge_201202/euslisp/main.l")

;; TODO: parameters should be arguments
(defvar *use-arm* :larm)
(defvar *use-base* nil)


;; define callbacks
(defun close-fridge-execute-cb (server goal)
  (close-fridge :use-arm *use-arm*))

(defun finish-execute-cb (server goal)
  (restore-params)
  t)

(defun finish-failure-execute-cb (server goal)
  (restore-params)
  t)

(defun grasp-can-execute-cb (server goal)
  (let ((can_name (send goal :goal :can_name)))
    (grasp-can :use-arm *use-arm*)))

(defun hand-over-can-execute-cb (server goal)
  (pass-to-human :use-arm *use-arm*))

(defun init-execute-cb (server goal)
  (let ((can_name (send goal :goal :can_name)))
    (ros::ros-warn "can name is ~A" can_name)
    (initialize-demo can_name)
    (store-params)
    (inflation-loose :local-costmap-inflation-radius 0.1
                     :global-costmap-inflation-radius 0.1)
    t))

(defun move-to-fridge-execute-cb (server goal)
  (goto-front-of-fridge))

(defun move-to-initial-position-execute-cb (server goal)
  (goto-initial-position :use-arm *use-arm*))

(defun open-fridge-execute-cb (server goal)
  (open-fridge-door :use-arm *use-arm* :use-base *use-base*))


;; create server instances
(instance roseus_bt:action-node :init
          "fridge_demo/close_fridge" fridge_demo_bt::CloseFridgeAction
          :execute-cb 'close-fridge-execute-cb)

(instance roseus_bt:action-node :init
          "fridge_demo/finish" fridge_demo_bt::FinishAction
          :execute-cb 'finish-execute-cb)

(instance roseus_bt:action-node :init
          "fridge_demo/finish_failure" fridge_demo_bt::FinishFailureAction
          :execute-cb 'finish-failure-execute-cb)

(instance roseus_bt:action-node :init
          "fridge_demo/grasp_can" fridge_demo_bt::GraspCanAction
          :execute-cb 'grasp-can-execute-cb)

(instance roseus_bt:action-node :init
          "fridge_demo/hand_over_can" fridge_demo_bt::HandOverCanAction
          :execute-cb 'hand-over-can-execute-cb)

(instance roseus_bt:action-node :init
          "fridge_demo/init" fridge_demo_bt::InitAction
          :execute-cb 'init-execute-cb)

(instance roseus_bt:action-node :init
          "fridge_demo/move_to_fridge" fridge_demo_bt::MoveToFridgeAction
          :execute-cb 'move-to-fridge-execute-cb)

(instance roseus_bt:action-node :init
          "fridge_demo/move_to_initial_position" fridge_demo_bt::MoveToInitialPositionAction
          :execute-cb 'move-to-initial-position-execute-cb)

(instance roseus_bt:action-node :init
          "fridge_demo/open_frirdge" fridge_demo_bt::OpenFridgeAction
          :execute-cb 'open-fridge-execute-cb)


;; set rate
(ros::rate 100)

;; spin
(roseus_bt:spin)
