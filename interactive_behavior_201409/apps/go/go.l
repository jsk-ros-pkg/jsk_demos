#!/usr/bin/env roseus
;; go.l

(require :app-utils "package://interactive_behavior_201409/euslisp/app-utils.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")

(defun pr2-init (&optional (create-viewer))
  (setq *pr2* (pr2)
        *ri* (instance pr2-interface :init
                       :move-base-action-name "elevator_move_base"))
  (ros::spin-once)
  (send *ri* :spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))


(defun main ()
  (let ((spot-name (get-arg :place)))
    (pr2-init)
    (setq *scene* (make-eng2-scene))

    (setq *spot* (get-spot spot-name))
    (unless *spot*
      (ros::ros-error "Destination '~A' not found" spot-name)
      (speak-jp "あー、ちょっとわからないですね。")
      (return-from main nil))
    (ros::ros-info "Going to ~A" (send *spot* :name))
    (speak-jp (format nil "~Aに行きます" (send *spot* :name)))

    (send *ri* :move-to *spot*)
    (return-from main t)))


(run-exit (main))
