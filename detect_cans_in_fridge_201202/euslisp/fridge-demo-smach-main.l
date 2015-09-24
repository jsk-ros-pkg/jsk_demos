#!/usr/bin/env irteusgl

(defvar *logging* nil)

(require "main.l")
(require :state-machine "package://roseus_smach/src/state-machine.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-mcahine-utils "package://roseus_smach/src/state-machine-utils.l")

(defclass fridge-demo-machine
 :super propertied-object
 )

(defmethod fridge-demo-machine
  (:init ()
         t)

  ;; goto-front-of-fridge
  (:goto-front-of-fridge
   (&key (wait t))
   (warning-message 2 ";; goto-front-of-fridge ~%")
   t)
  ;; Internal function of goto-front-of-fridge
  (:base-go-to-fridge
   (&key (wait t))
   (warning-message 2 ";; base-go-to-fridge ~%")
   (base-go-to-fridge :wait wait)
   t)
  (:base-go-to-fridge-init-pose
   (&key (wait t))
   (warning-message 2 ";; base-go-to-fridge-init-pose ~%")
   (base-go-to-fridge-init-pose :wait wait)
   t)

  ;;open-fridge-door
  (:open-fridge-door
   (&key (wait t))
   (warning-message 2 ";; open-fridge-door ~%")
   t)
  ;; Internal function of open-fridge-door
  (:open-fridge-door-initial-pose
   (&key (wait t))
   (warning-message 2 ";; open-fridge-door-initial-pose ~%")
   (open-fridge-door-initial-pose :door-type :circle :wait wait)
   t)
  (:move-to-and-open-fridge-door
   (&key (wait t))
   (warning-message 2 ";; move-to-and-open-fridge-door ~%")
   (move-to-and-open-fridge-door :door-type :circle :move t
                                 :open-fridge-func #'open-fridge-traj :wait wait)
   t)
  (:move-forward-larm
   (&key (wait t))
   (warning-message 2 ";; move-forward-larm ~%")
   (move-forward-larm :wait wait)
   t)
  (:detach-fridge-handle
   (&key (wait t))
   (warning-message 2 ";; detach-fridge-handle ~%")
   (detach-fridge-handle :wait wait)
   t)
  (:swipe-fridge-door
   (&key (wait t))
   (warning-message 2 ";; swipe-fridge-door ~%")
   (swipe-fridge-door :door-type :circle :use-arm *use-arm* :wait wait)
   t)

  ;; grasp-can
  (:grasp-can
   (&key (wait t))
   (warning-message 2 ";; grasp-can ~%")
   t)
  ;; Internal function of grasp-can
  (:move-to-can-spot
   (&key (wait t))
   (warning-message 2 ";; move-to-can-spot ~%")
   (move-to-can-spot :use-arm *use-arm*)
   t)
  (:grasp-can-init
   (&key (wait t))
   (warning-message 2 ";; grasp-can-init ~%")
   (grasp-can)
   t)
  (:grasp-can-motion
   (&key (wait t))
   (warning-message 2 ";; grasp-can-motion ~%")
   (grasp-can-motion :use-arm *use-arm*)
   t)
  (:go-back-from-fridge
   (&key (wait t))
   (warning-message 2 ";; go-back-from-fridge ~%")
   ;; (go-back-from-fridge) ;; not defined
   t)

  ;; close-fridge
  (:close-fridge
   (&key (wait t))
   (warning-message 2 ";; close door ~%")
   t)
  ;; Internal function of close-fridge
  (:close-fridge-motion
   (&key (wait t))
   (warning-message 2 ";; close-fridge-motion ~%")
   ;; (close-fridge-motion) ;; not defined
   t)
  (:tuckarm
   (&key (wait t))
   (warning-message 2 ";; tuckarm ~%")
   (pr2-tuckarm-pose *use-arm* :outside)
   t)
  )

(defun make-goto-front-of-fridge ()
  (setq *goto-fridge-sm*
        (make-state-machine
         '((:base-go-to-fridge -> :base-go-to-fridge-init-pose)
           (:base-go-to-fridge-init-pose -> t))
         '((:base-go-to-fridge #'(lambda (&rest args) (send *fridge-demo* :base-go-to-fridge :wait *wait*)))
           (:base-go-to-fridge-init-pose #'(lambda (&rest args) (send *fridge-demo* :base-go-to-fridge-init-pose :wait *wait*))))
         '(:base-go-to-fridge)
         '(t))))

(defun make-open-fridge-door-state-machine ()
  (setq *open-fridge-sm*
        (make-state-machine
         '((:open-fridge-door-initial-pose -> :move-to-and-open-fridge-door) ;; should do 10 times
           (:move-to-and-open-fridge-door -> :move-forward-larm)
           (:move-forward-larm -> :detach-fridge-handle)
           (:detach-fridge-handle -> :swipe-fridge-door)
           (:swipe-fridge-door -> t))
         '((:open-fridge-door-initial-pose #'(lambda (&rest args) (send *fridge-demo* :open-fridge-door-initial-pose :wait *wait*)))
           (:move-to-and-open-fridge-door #'(lambda (&rest args) (send *fridge-demo* :move-to-and-open-fridge-door :wait *wait*)))
           (:move-forward-larm #'(lambda (&rest args) (send *fridge-demo* :move-forward-larm)))
           (:detach-fridge-handle #'(lambda (&rest args) (send *fridge-demo* :detach-fridge-handle :wait *wait*)))
           (:swipe-fridge-door #'(lambda (&rest args) (send *fridge-demo* :swipe-fridge-door :wait *wait*))))
         '(:open-fridge-door-initial-pose)
         '(t))))

(defun make-grasp-can-state-machine ()
  (setq *grasp-can-sm*
        (make-state-machine
         '((:move-to-can-spot -> :grasp-can-init)
           (:grasp-can-init -> :grasp-can-motion) ;; should do 10 times
           (:grasp-can-motion -> :go-back-from-fridge) ;; go back func is not defined yet.
           (:go-back-from-fridge -> t))
         '((:move-to-can-spot #'(lambda (&rest args) (send *fridge-demo* :move-to-can-spot :wait *wait*)))
           (:grasp-can-init #'(lambda (&rest args) (send *fridge-demo* :grasp-can-init :wait *wait*)))
           (:grasp-can-motion #'(lambda (&rest args) (send *fridge-demo* :grasp-can-motion :wait *wait*)))
           (:go-back-from-fridge #'(lambda (&rest args) (send *fridge-demo* :go-back-from-fridge :wait *wait*))))
         '(:move-to-can-spot)
         '(t))))

(defun make-close-fridge-state-machine ()
  (setq *close-fridge-sm*
        (make-state-machine
         '((:close-fridge-motion -> :tuckarm) ;; close-fridge-motion func is not defined yet.
           (:tuckarm -> t))
         '((:close-fridge-motion #'(lambda (&rest args) (send *fridge-demo* :grasp-can-init :wait *wait*)))
           (:tuckarm #'(lambda (&rest args) (send *fridge-demo* :tuckarm :wait *wait*))))
         '(:close-fridge-motion)
         '(t))))

(defun make-fridge-demo-state-machine ()
  (setq *sm*
        (make-state-machine
         '((:goto-front-of-fridge -> :open-fridge-door)
           (:open-fridge-door ->  :grasp-can)
           (:grasp-can -> :close-fridge)
           (:close-fridge -> :success))
         '((:goto-front-of-fridge *goto-fridge-sm*)
           (:open-fridge-door *open-fridge-sm*)
           (:grasp-can *grasp-can-sm*)
           (:close-fridge *close-fridge-sm*))
         '(:goto-front-of-fridge)
         '(:success))))

(defun init ()
  (ros::roseus "sample_parallel_state_machine")
  (setq *use-arm* :larm)
  (setq *fridge-demo* (instance fridge-demo-machine :init))
  (make-goto-front-of-fridge)
  (make-open-fridge-door-state-machine)
  (make-grasp-can-state-machine)
  (make-close-fridge-state-machine)
  (make-fridge-demo-state-machine)
  (ros::ros-info "created state machine ~A" *sm*))

(warning-message 3 ";; (init)~%")

(defun demo (&optional (wait t))
  (setq *wait* wait)
  (when (or (not (boundp '*sm*)) (not (boundp '*fridge-demo*)))
    (init))
  (exec-smach-with-spin *sm*))

(warning-message 3 ";; (demo)~%")

(provide :fridge-demo-machine)
