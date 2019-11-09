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
      (ros::ros-info "Received sandwich.")
      (speak-jp "ありがとうございます。" :wait t)
      (return-from wait-for-hand-over t))
    (ros::ros-warn
     (format nil "Hand impact timed out (~A sec) without response" timeout))
    (speak-jp "サンドイッチを手にかけてください。" :wait t))
  ;; fail
  (speak-jp "あれ、サンドイッチ。。。" :wait t)
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
  (setq *initial-pose* (send *ri* :state :worldcoords)))


(defun go-to-forum-door-inside ()
  (clear-costmap)
  (send *ri* :move-to
        (send *scene* :spot "/eng2/2f/forum-door-inside"))
  (update-eus-robot-model))


(defun check-forum-door-open (&optional (max-cloud-size 500))
  (let (is-door-open cloud-msg)
    (send *pr2* :head :look-at
          (send (send *scene* :spot "/eng2/2f/forum-door-button") :worldpos))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation)
    (unix:sleep 1)
    ;; Assume the point cloud to be unorganized and is_dense: True.
    (ros::spin-once)
    (dotimes (i 3)
      (setq cloud-msg
            (one-shot-subscribe "extract_eng2_forum_door_point_cloud/extract_indices/output"
                                sensor_msgs::PointCloud2
                                :timeout 10000)))  ;; 10000[msec]
    (unless cloud-msg
      (return-from check-forum-door-open nil))
    (setq is-door-open
          (<= (* (send cloud-msg :width) (send cloud-msg :height))
              max-cloud-size))
    (ros::ros-info
     (format nil "number of points: ~A, max-cloud-size: ~A, is-door-open: ~A"
             (* (send cloud-msg :width) (send cloud-msg :height))
             max-cloud-size
             is-door-open))
    (when is-door-open
      (ros::ros-info "Door in front of forum is open.")
      (speak-jp "ドアが開いています。" :wait t)
      (return-from check-forum-door-open t))
    (ros::ros-info "Door in front of forum is closed.")
    (speak-jp "ドアが閉まっています。" :wait t)
    (return-from check-forum-door-open nil)
    ))


(defun open-forum-door ()
  (let (bboxes-msg bbox cam->button-coords cam-coords forum-door-button-pos
                   target-coords ray-vec via-coords push-coords
                   av0 av1 (arm :rarm))

    ;; Find door button
    (send *pr2* :head :look-at
          (send (send *scene* :spot "/eng2/2f/forum-door-button") :worldpos))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation)
    (unix:sleep 1)
    (ros::ros-info "Looking for door button...")
    (speak-jp "ドアのボタンを探しています。" :wait t)
    (update-eus-robot-model)
    (ros::spin-once)
    (dotimes (i 3)
      (setq bboxes-msg
            (one-shot-subscribe "door_button/cluster_indices_decomposer/boxes"
                                jsk_recognition_msgs::BoundingBoxArray
                                :timeout 10000)))  ;; 10000[msec]
    (when (or (null bboxes-msg) (null (send bboxes-msg :boxes)))
      (return-from open-forum-door nil))
    (setq bbox (elt (send bboxes-msg :boxes) 0))  ;; first (expected biggest)
    (setq cam->button-coords (ros::tf-pose->coords (send bbox :pose)))
    (setq cam-coords (send (send *pr2* :head_mount_kinect_rgb_optical_frame_lk)
                           :copy-worldcoords))
    (setq forum-door-button-pos
          (send (send (send cam-coords :copy-worldcoords)
                      :transform cam->button-coords)
                :worldpos))
    (setq target-coords (make-coords :pos forum-door-button-pos))

    ;; Define via-coords (70mm in front of button) and push-coords (push 30mm)
    (setq ray-vec
          (normalize-vector
           (v- (float-vector
                (elt (send *pr2* :laser_tilt_mount_link_lk :worldpos) 0)
                (elt (send *pr2* :laser_tilt_mount_link_lk :worldpos) 1)
                (elt (send target-coords :worldpos) 2))
               (send target-coords :worldpos))))
    (setq via-coords (send (send target-coords :copy-worldcoords)
                           :translate (scale 70 ray-vec) :world))
    (setq push-coords (send (send target-coords :copy-worldcoords)
                            :translate (scale -30 ray-vec) :world))

    ;; Push door button
    (ros::ros-info "Pushing button...")
    (speak-jp "ドアのボタンを押します。" :wait t)
    (send *ri* :start-grasp arm)
    (pr2-reset-pose)
    (setq av0 (send *pr2* arm :inverse-kinematics via-coords
                    :rotation-axis :x
                    :look-at-target t))
    (setq av1 (send *pr2* arm :inverse-kinematics push-coords
                    :rotation-axis :x
                    :look-at-target t))
    (send *ri* :angle-vector-sequence (list av0 av1) (list 3000 1000))
    (send *ri* :wait-interpolation)
    (pr2-tuckarm-pose)
    (update-eus-robot-model)
  ))


(defun go-to-subway-front ()
  (clear-costmap)
  (send *ri* :move-to
        (send *scene* :spot "/eng2/2f/subway-front")))


(defun order-sandwich (name)
  (ros::ros-info "Ordering sandwich...")
  (speak-jp "すみません。" :wait t)
  (speak-jp (format nil "~Aをください。" name) :wait t))


(defun receive-sandwich ()
  ;; go to register front
  (send *ri* :move-to
        (send *scene* :spot "/eng2/2f/subway-register"))

  ;; TODO(Unknown): Pay money here.
  ;;                Consider registering SUBWAY club and getting pre-paid card.

  (send *pr2* :angle-vector (pr2-pick-bag-pose))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm :wait t)

  (ros::ros-info "Waiting for receiving sandwich.")
  (speak-jp "商品をください。" :wait t)

  (wait-for-hand-over :rarm 60)

  (send *ri* :start-grasp :rarm)

  (speak-jp "後ろに下がります。気をつけてください。" :wait t)
  (clear-costmap)
  (send *ri* :go-pos-unsafe -0.5 0 0)

  (send *pr2* :torso_lift_joint :joint-angle 0)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (pr2-tuckarm-pose :rarm :outside)

  (send *ri* :go-pos-unsafe 0 0 180)
  t)


(defun go-to-initial-pose ()
  (ros::ros-info "Coming back to original position...")
  (speak-jp "もとの場所に帰ります。" :wait t)
  ;; NOTE: DWA local planner returns invalid plan when detecting door closed.
  (ros::ros-info "Move to /eng2/2f/forum-door-outside.")
  (send *ri* :move-to (send *scene* :spot "/eng2/2f/forum-door-outside"))
  (ros::ros-info (format nil "Move to ~A" *initial-pose*))
  (send *ri* :move-to *initial-pose*))


(defun pass-sandwich (name)
  (send *pr2* :angle-vector (pr2-pick-bag-pose))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :rarm :wait t)
  (speak-jp (format nil "~Aを買ってきました。" name) :wait t)

  (while (not (wait-for-hand-impact :rarm :timeout 30))
    (ros::row-info "Still waiting for a human to get sandwich from PR2.")
    (speak-jp (format nil "~Aをどうぞ。" name) :wait t))

  (send *ri* :start-grasp :rarm)
  (send *pr2* :torso_lift_joint :joint-angle 0)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (pr2-tuckarm-pose))


(defun demo (&optional (name "BLTサンド"))
  (ros::roseus "fetch_sandwich")
  (speak-jp (format nil "~Aを買いに行きます。" name) :wait t)
  (init)
  (go-to-forum-door-inside)
  (while (not (check-forum-door-open))
    (open-forum-door)
    (unix:sleep 1)  ;; Wait for door opening
    )
  (go-to-subway-front)
  (order-sandwich name)
  (receive-sandwich)
  (go-to-initial-pose)
  (pass-sandwich name))


(when (find #'(lambda (x) (substringp "__name:=" x)) lisp::*eustop-argument*)
  (demo))
