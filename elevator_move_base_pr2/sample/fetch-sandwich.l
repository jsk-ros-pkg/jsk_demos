#!/usr/bin/env roseus
;; fetch-sandwich.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :navigation-client "package://elevator_move_base_pr2/src/navigation-client.l")


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; PR2 Utility Functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun pr2-init (&optional (create-viewer))
  (setq *pr2* (pr2)
        *ri* (instance pr2-interface :init
                       :move-base-action-name "elevator_move_base"))
  (ros::spin-once)
  (send *ri* :spin-once)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (when create-viewer (objects (list *pr2*))))

(defun pr2-pick-bag-pose ()
  #|
  (send *pr2* :reset-pose)
  (send *pr2* :torso :waist-z :joint-angle 250.0)
  (let ((cds (make-cascoords :pos (float-vector 800 0 1300))))
    (send cds :rotate pi/2 :x)
    (send *pr2* :rarm :inverse-kinematics
          cds
          :use-torso t
          :look-at-target t))
  |#
  (float-vector 250.0 60.0 74.0 70.0 -120.0 20.0 -30.0 180.0 -13.5232 -15.8808 -82.6112 -83.1698 -162.008 -72.0918 170.603 0.0 40.2852))

(defun wait-for-hand-over (arm &optional (timeout 10))
  (dotimes (i 3)
    (when (wait-for-hand-impact arm :timeout timeout)
      (speak-jp "ありがとうございます。")
      (return-from wait-for-hand-over t))
    (ros::ros-error "Hand impact timed out without response")
    (speak-jp "サンドイッチを手にかけてください。"))
  ;; fail
  (speak-jp "あれ、サンドイッチ。。。")
  (return-from wait-for-hand-over nil))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Actions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun init ()
  (pr2-init)
  (setq *scene* (make-eng2-scene))
  (setq *initial-pose* (send *ri* :state :worldcoords))
  (setq *initial-floor* (check-current-floor *initial-pose*)))

(defun go-to-subway-front ()
  (clear-costmap)
  (change-inflation-range 0.4)
  (send *ri* :move-to
        (send *scene* :spot "/eng2/2f/subway-front")))

(defun order-sandwich (&optional (name "BLTサンド"))
  (speak-jp "すみません")
  (speak-jp (format nil "~Aをください" name)))

(defun receive-sandwich ()
  ;; go to register front
  (send *ri* :move-to
        (send *scene* :spot "/eng2/2f/subway-register"))

  (send *pr2* :angle-vector (pr2-pick-bag-pose))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm :wait t)

  (speak-jp "サンドイッチをください")

  (wait-for-hand-over :rarm)

  (send *ri* :start-grasp :rarm)

  (speak-jp "後ろに下がります。気をつけてください" :wait t)
  (send *ri* :go-pos-unsafe -0.5 0 0)

  (pr2-tuckarm-pose :rarm :outside)
  t)

(defun go-to-initial-pose ()
  (send *ri* :move-to
        *initial-pose*
        :frame-id *initial-floor*))


(defun pass-sandwich ()
  (send *pr2* :angle-vector (pr2-pick-bag-pose))
  (send *ri* :angle-vector (send *pr2* :angle-vector 5000))
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :rarm :wait t)
  (speak-jp "サンドイッチをどうぞ")

  (wait-for-hand-over :rarm)

  (send *ri* :start-grasp :rarm)
  (pr2-tuckarm-pose))

(defun demo ()
  (init)
  (go-to-subway-front)
  (order-sandwich)
  (receive-sandwich)
  (go-to-initial-pose)
  (pass-sandwich))

(when (find #'(lambda (x) (substringp "__name:=" x)) lisp::*eustop-argument*)
  (demo))
