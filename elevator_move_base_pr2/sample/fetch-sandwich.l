#!/usr/bin/env roseus
;; -*- coding: utf-8 -*-
;; fetch-sandwich.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :navigation-client "package://elevator_move_base_pr2/src/navigation-client.l")

(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "sensor_msgs")


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


(defun update-eus-robot-model ()
  (ros::spin-once)
  (send *pr2* :move-to (send *ri* :state :worldcoords) :world)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Actions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun init ()
  (pr2-init)
  (setq *scene* (make-eng2-scene))
  (setq *initial-pose* (send *ri* :state :worldcoords))
  (setq *initial-floor* (check-current-floor *initial-pose*)))


(defun go-to-forum-door-front ()
  (clear-costmap)
  (change-inflation-range 0.4)
  (send *ri* :move-to
        (send *scene* :spot "/eng2/2f/forum-door-front"))
  (update-eus-robot-model))


(defun check-forum-door-open (&optional (max-cloud-size 500))
  (let (is-door-open cloud-msg)
    (dotimes (i 3)
      (unix:sleep 2)
      (setq cloud-msg
            (one-shot-subscribe "forum_door/extract_indices/output"
                                sensor_msgs::PointCloud2))
      (setq is-door-open (<= (length (send cloud-msg :data)) max-cloud-size))
      (when (is-door-open)
        (speak-jp "ドアが開いています。")
        (return-from check-door-open t))
      )
    (speak-jp "ドアが閉まっています。")
    (return-from check-forum-door-open nil)
    ))


(defun open-forum-door ()
  (let (bboxes-msg bbox cam->button-coords cam-coords forum-door-button-pos
        target-coords ray-vec via-coords push-coords av0 av1 (arm :rarm))

    ;; Find door button
    (speak-jp "ドアのボタンを探しています。")
    (send *pr2* :head :look-at
          (send (send *scene* :spot "/eng2/2f/forum-door-button") :worldpos))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (update-eus-robot-model)
    (setq bboxes-msg
          (one-shot-subscribe "door_button/cluster_indices_decomposer/boxes"
                              jsk_recognition_msgs::BoundingBoxArray
                              :timeout 10000))  ;; 10000[msec]
    (unless bboxes-msg
      (return-from open-forum-door nil))
    (setq bbox (elt (send bboxes-msg :boxes) 0))  ;; first (expected biggest)
    (setq cam->button-coords (ros::tf-pose->coords (send bbox :pose)))
    (setq cam-coords (send (send *pr2* :head_mount_kinect_rgb_optical_frame_lk)
                           :copy-worldcoords))
    (setq forum-door-button-pos
          (scale 0.001 (send (send cam-coords :transform cam->button-coords)
                             :worldpos)))
    (setq target-coords (make-coords :pos forum-door-button-pos))

    ;; Define via-coords and push-coords
    (setq ray-vec
          (normalize-vector
           (v- (send *pr2* :head_mount_kinect_rgb_optical_frame_lk :worldpos)
               (send target-coords :worldpos))))
    (setq via-coords (send (send target-coords :copy-worldcoords)
                           :translate (scale 50 ray-vec) :world))
    (setq push-coords (send (send target-coords :copy-worldcoords)
                            :translate (scale -30 ray-vec) :world))

    ;; Push door button
    (speak-jp "ドアのボタンを押します")
    (setq av0 (send *pr2* arm :inverse-kinematics via-coords
                    :rotation-axis :x
                    :use-torso t
                    :look-at-target t))
    (setq av1 (send *pr2* arm :inverse-kinematics push-coords
                    :rotation-axis :x
                    :use-torso t
                    :look-at-target t))
    (send *ri* :stop-grasp arm)
    (send *ri* :angle-vector-sequence (list av0 av1) (list 3000 1000))
    (send *ri* :wait-interpolation)
    (pr2-tuckarm-pose)
    (update-eus-robot-model)
  ))


(defun go-to-subway-front ()
  (clear-costmap)
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

  (send *pr2* :torso_lift_joint :joint-angle 0)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)

  (pr2-tuckarm-pose :rarm :outside)
  t)


(defun go-to-initial-pose ()
  (send *ri* :move-to
        (send (send *initial-pose* :copy-worldcoords) :translate #f(0 0 -30000))
        :frame-id *initial-floor*))


(defun pass-sandwich ()
  (send *pr2* :angle-vector (pr2-pick-bag-pose))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :rarm :wait t)
  (speak-jp "サンドイッチをどうぞ")

  (wait-for-hand-over :rarm)

  (send *ri* :start-grasp :rarm)
  (pr2-tuckarm-pose))


(defun demo ()
  (speak-jp "サンドイッチを買いに行きます")
  (init)
  (go-to-forum-door-front)
  (while (not (check-forum-door-open))
    (open-forum-door)
    (unix:sleep 2)  ;; Wait for door opening
    )
  (go-to-subway-front)
  (order-sandwich)
  (receive-sandwich)
  (go-to-initial-pose)
  (pass-sandwich))


(when (find #'(lambda (x) (substringp "__name:=" x)) lisp::*eustop-argument*)
  (demo))
