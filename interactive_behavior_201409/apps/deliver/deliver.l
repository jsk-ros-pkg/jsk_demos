#!/usr/bin/env roseus
;; deliver.l

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
  t)

(if (main) (exit 0) (exit 1))
