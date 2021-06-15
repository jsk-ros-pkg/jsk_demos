#!/usr/bin/env roseus
;; go-to-room.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")


(defvar *pr2*)
(defvar *ri*)
(defvar *scene*)
(defvar *spot*)
(defvar *spot-name*)


(defun pr2-init (&optional (create-viewer))
  (setq *pr2* (pr2)
        *ri* (instance pr2-interface :init
                       :move-base-action-name "elevator_move_base"))
  (ros::spin-once)
  (send *ri* :spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (when create-viewer (objects (list *pr2*))))

(defun main ()
  (ros::roseus "pr2_go_to_room")

  (pr2-init)
  (setq *scene* (make-eng2-scene))

  (setq *spot-name* (ros::get-param "~destination" ""))
  (setq *spot* (send *scene* :spot *spot-name*))
  (unless *spot*
    (ros::ros-error "Destination '~A' not found" *spot-name*)
    (return-from main))

  (send *ri* :move-to *spot*)
  (return-from main t))

(main)
