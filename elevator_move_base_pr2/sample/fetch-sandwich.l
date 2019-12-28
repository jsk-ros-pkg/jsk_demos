#!/usr/bin/env roseus
;; -*- coding: utf-8 -*-
;; fetch-sandwich.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-srvs "std_srvs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :app-utils "package://jsk_demo_common/euslisp/app-utils.l")
(require :pr2-action "package://jsk_demo_common/euslisp/pr2-action.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")

(load "package://elevator_move_base_pr2/src/utils.l")


(defvar *initial-pose*)
(defvar *pr2*)
(defvar *ri*)
(defvar *scene*)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; PR2 Utility Functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun look-front ()
  (send *pr2* :head :angle-vector #f(0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation))


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
  (float-vector 325.0 60.0 74.0 70.0 -120.0 20.0 -30.0 180.0 -13.5232 -15.8808
                -82.6112 -83.1698 -162.008 -72.0918 170.603 0.0 40.2852))


(defun wait-for-hand-over (arm &optional (timeout 10) (n-trial 3))
  (dotimes (i n-trial)
    (when (wait-for-hand-impact arm :timeout timeout)
      (ros::ros-info "Received sandwich.")
      (speak-jp "ありがとうございます。" :wait t)
      (return-from wait-for-hand-over t))
    (ros::ros-warn
     (format nil "Hand impact timed out (~A sec) without response" timeout))
    (speak-jp "袋を私の手に掛けて、手を揺らしてください。" :wait t))
  ;; fail
  (speak-jp "あれ、サンドイッチ。。。" :wait t)
  (return-from wait-for-hand-over nil))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Actions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun init ()
  (setq *pr2* (pr2))
  (setq *ri* (instance pr2-interface :init
                       :move-base-action-name "elevator_move_base")))


(defun go-to-forum-door-inside ()
  (clear-costmap)
  ;; XXX: Avoid getting stuck on braille block (bumpy tiles for the blind)
  ;;      at elevator hall on 2F.
  (send *ri* :move-to
        (send (send (send *scene* :spot "/eng2/2f/forum-door-inside")
                    :copy-worldcoords) :translate #f(-3200 -100 0) :world))
  (send *ri* :go-pos-unsafe 3.2 0 0)
  ;; Modify position
  (send *ri* :move-to (send *scene* :spot "/eng2/2f/forum-door-inside"))
  (update-robot-position))


(defun check-forum-door-open (&optional (max-cloud-size 500) (timeout 10000))
  (let (is-door-open cloud-msg)
    (send *pr2* :head :look-at
          (send (send *scene* :spot "/eng2/2f/forum-door-button") :worldpos))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation)
    (ros::ros-info "Checking if door in front of forum is open...")
    ;; Assume the point cloud to be unorganized and is_dense: True.
    (setq cloud-msg
          (one-shot-subscribe
           "extract_eng2_forum_door_point_cloud/extract_indices/output"
           sensor_msgs::PointCloud2
           :timeout timeout  ;; timeout[msec]
           :after-stamp (ros::time+ (ros::time-now) (ros::time 1.0))))
    (unless cloud-msg
      (ros::ros-warn
       (format nil "No point cloud was received in ~A[msec]." timeout))
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
      (speak-jp "ドアが開いています。")
      (return-from check-forum-door-open t))
    (ros::ros-info "Door in front of forum is closed.")
    (speak-jp "ドアが閉まっています。")
    (return-from check-forum-door-open nil)
    ))


(defun push-forum-door-button (&optional (timeout 15000))
  (let (bboxes-msg bbox cam->button-coords cam-coords forum-door-button-pos
                   target-coords ray-vec via-coords push-coords
                   av0 av1 (arm :rarm))
    ;; Find door button
    (ros::ros-info "Looking for door button...")
    (speak-jp "ボタンを探しています。")
    (update-robot-position)
    (setq bboxes-msg
          (one-shot-subscribe
           "door_button/cluster_indices_decomposer/boxes"
           jsk_recognition_msgs::BoundingBoxArray
           :timeout timeout  ;; timeout[msec]
           :after-stamp (ros::time+ (ros::time-now) (ros::time 0.5))))
    (when (or (null bboxes-msg) (null (send bboxes-msg :boxes)))
      (ros::ros-warn
       (format nil "Button bounding box was not found in ~A[msec]." timeout))
      (return-from push-forum-door-button nil))
    (setq bbox (elt (send bboxes-msg :boxes) 0))  ;; first (expected biggest)
    (setq cam->button-coords (ros::tf-pose->coords (send bbox :pose)))
    (setq cam-coords (send (send *pr2* :head_mount_kinect_rgb_optical_frame_lk)
                           :copy-worldcoords))
    (setq forum-door-button-pos
          (send (send (send cam-coords :copy-worldcoords)
                      :transform cam->button-coords)
                :worldpos))
    (setq target-coords (make-coords :pos forum-door-button-pos))

    ;; Define via-coords (70mm in front of button) and push-coords (push 10mm)
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
                            :translate (scale -10 ray-vec) :world))

    (ros::ros-info "Pushing button...")
    (speak-jp "ボタンを押します。")

    ;; We have to reset arm pose before pushing button.
    (send *ri* :start-grasp arm)
    (send *pr2* :reset-pose)
    (send *pr2* :head :look-at
          (send (send *scene* :spot "/eng2/2f/forum-door-button") :worldpos))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)

    ;; Push door button
    (unless (and (setq av0 (send *pr2* arm :inverse-kinematics via-coords
                                 :rotation-axis :x
                                 :look-at-target t))
                 (setq av1 (send *pr2* arm :inverse-kinematics push-coords
                                 :rotation-axis :x
                                 :look-at-target t)))
      (return-from push-forum-door-button nil))
    (send *ri* :angle-vector-sequence (list av0 av1 av0) (list 3000 2000 1000))
    (send *ri* :wait-interpolation)
    (pr2-tuckarm-pose)
    (update-robot-position)
  ))


(defun open-forum-door ()
  (call-service "door_button/pass_through_image/request"
                (instance std_srvs::EmptyRequest :init))
  (until (check-forum-door-open)
         (push-forum-door-button)
         (unix:sleep 1))  ;; Wait for door opening
  (call-service "door_button/pass_through_image/stop"
                (instance std_srvs::EmptyRequest :init))
  (look-front))


(defun go-to-subway-front ()
  (clear-costmap)
  ;; TODO(Unknown): Wait in line.
  (send *ri* :move-to
        (send *scene* :spot "/eng2/2f/subway-front")
        :retry 100))


(defun order-sandwich
    (sandwich
     &key (bread "オススメ") (topping "オススメ")
          (vegetable "オススメ") (dressing "オススメ"))
  (ros::ros-info "Looking for clerk...")
  (send *pr2* :head :angle-vector #f(-25 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 500 :head-controller)
  (call-service "look_at_human/start" (instance std_srvs::EmptyRequest :init))
  (unix:sleep 5)
  (ros::ros-info "Ordering sandwich...")
  (ros::ros-info (format nil "Sandwich: ~A" sandwich))
  (ros::ros-info (format nil "Bread: ~A" bread))
  (ros::ros-info (format nil "Topping: ~A" topping))
  (ros::ros-info (format nil "Vegetable: ~A" vegetable))
  (ros::ros-info (format nil "Dressing: ~A" dressing))
  (ros::ros-info "Side Menu: nil")
  (ros::ros-info "Others: leave it to the clerk")
  (ros::ros-info "To go (Take away)")
  (speak-jp "こんにちは。おつかいに来ました。" :wait t)
  (speak-jp (format nil "~Aをください。" sandwich) :wait t)
  (speak-jp (format nil "パンの種類は、~Aでお願いします。" bread) :wait t)
  (speak-jp (format nil "トッピングは、~Aでお願いします。" topping) :wait t)
  (speak-jp (format nil "野菜は、~Aでお願いします。" vegetable) :wait t)
  (speak-jp (format nil "ドレッシングは、~Aでお願いします。" dressing) :wait t)
  (speak-jp "サイドメニューは、要りません。" :wait t)
  (speak-jp "あとは、おまかせでお願いします。" :wait t)
  (speak-jp "サンドイッチは、持ち帰ります。" :wait t)  ;; Reduce tax (2019/10/1~)
  (call-service "look_at_human/stop" (instance std_srvs::EmptyRequest :init)))


(defun receive-sandwich ()
  ;; Go to register front
  (clear-costmap)
  (send *ri* :move-to
        (send (send (send *scene* :spot "/eng2/2f/subway-register")
                    :copy-worldcoords) :translate #f(0 -200 0) :world)
        :retry 100)

  ;; TODO(Unknown): Pay money here.
  ;;                Consider registering SUBWAY club and getting pre-paid card.

  (send *pr2* :angle-vector (pr2-pick-bag-pose))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; Go to register
  (speak-jp "少し前に出ます。" :wait t)
  (send *ri* :go-pos-unsafe 0.2 0 0)

  (send *ri* :stop-grasp :rarm :wait t)

  (ros::ros-info "Waiting for receiving sandwich.")
  (speak-jp "商品をください。" :wait t)
  (speak-jp "袋を私の手に掛けて、手を揺らしてください。" :wait t)

  (wait-for-hand-over :rarm 60 10)  ;; wait at most 10 min

  (send *ri* :start-grasp :rarm)

  (speak-jp "後ろに下がります。気をつけてください。" :wait t)
  (clear-costmap)
  (send *ri* :go-pos-unsafe -0.5 0 0)

  (send *pr2* :torso_lift_joint :joint-angle 0)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; Prevent sandwich from covering /base_scan region.
  (pr2-tuckarm-pose :rarm)
  (look-front)

  (send *ri* :go-pos-unsafe 0 0 180)
  t)


(defun go-to-initial-pose ()
  (ros::ros-info "Coming back to original position...")
  (speak-jp "もとの場所に帰ります。" :wait t)
  ;; NOTE: DWA local planner does not publish command to go through the door
  ;;       when detecting door closed, so move to forum-door-outside first
  ;;       in order to approach the door.
  (ros::ros-info "Move to /eng2/2f/forum-door-outside.")
  (send *ri* :move-to (send *scene* :spot "/eng2/2f/forum-door-outside"))
  (ros::ros-info (format nil "Move to initial pose: ~A." *initial-pose*))
  (call-service "look_at_human/start" (instance std_srvs::EmptyRequest :init))
  ;; NOTE: Robot does not have student ID card, so please open door when needed.
  (send *ri* :move-to *initial-pose*))


(defun pass-sandwich (sandwich)
  (call-service "look_at_human/stop" (instance std_srvs::EmptyRequest :init))
  (update-robot-position)
  (send *ri* :go-pos-unsafe 0 0 (send *pr2* :head :neck-y :joint-angle))
  (look-front)

  (send *pr2* :angle-vector (pr2-pick-bag-pose))
  (send *pr2* :head :angle-vector #f(0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :rarm :wait t)
  (speak-jp (format nil "~Aを買ってきました。" sandwich) :wait t)

  (until (wait-for-hand-impact :rarm :timeout 30)
         (ros::ros-info "Still waiting for a human to get sandwich from PR2.")
         (speak-jp (format nil "~Aをどうぞ。" sandwich) :wait t))

  (send *ri* :start-grasp :rarm)
  (send *pr2* :torso_lift_joint :joint-angle 0)
  (look-front)
  (pr2-tuckarm-pose))


(defun demo (&key (sandwich "BLTサンド"))
  (send *pr2* :reset-pose)
  (pr2-tuckarm-pose)
  (look-front)
  (setq *scene* (make-eng2-scene))
  (setq *initial-pose* (send *ri* :state :worldcoords))
  (speak-jp (format nil "~Aを買いに行きます。" sandwich) :wait t)
  (go-to-forum-door-inside)
  (open-forum-door)
  (go-to-subway-front)
  (order-sandwich sandwich)
  (receive-sandwich)
  (go-to-initial-pose)
  (pass-sandwich sandwich))


(defun main (&key (app-manager :false))
  (ros::roseus "fetch_sandwich")
  (init)
  (if (eq app-manager :true)
      (progn
        (let ((sandwich (string-downcase (get-arg :sandwich))))
          (cond
           ((find sandwich
                  (list "BLTサンド" "ローストチキンサンド" "ツナサンド")
                  ;; 390 * 1.08 = 421 yen (2019/12/27)
                  :test #'string=)
            (demo :sandwich sandwich))
           (t
            (speak-jp "ごめんなさい。今ちょっと手が離せません。" :wait t)))))
    (demo))
  (exit 0))
