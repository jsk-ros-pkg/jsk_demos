;;棚から箱を取ってくる動作

;;load packages
(load "package://fetcheus/fetch-interface.l")
(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
;;(load "models/73b2-cupboard-without-door-object.l") ;;棚
(load "./shelf_model.l") ;;自作棚

(ros::load-ros-manifest "jsk_recognition_msgs")

;;defvarは一度やってしまったら上書きされない
(defvar *topic-name* "/head_camera_remote/depth_registered/boxes")
(defvar *gift-name* "/atohayoroshiku")
(defvar *bounding-box-list* nil)
(defvar *gift-number* nil)

(fetch-init)
(objects (list *fetch*))

(setq time-stamp (ros::time 0))

;;callback
(defun bounding-box-array-cb (msg)
  ;;(ros::ros-info "cb called")
  (setq *bounding-box-list* (send msg :boxes)) ;; boxesは、BoundingBoxのArray(Euslispではlist)
  ;;(ros::ros-info "set box-list")
  (when *bounding-box-list*
    (let* ((b (elt *bounding-box-list* 0))
           (base->obj-coords (ros::tf-pose->coords (send b :pose)))
           (base-coords (send (send *fetch* :base_link_lk) :copy-worldcoords)))
      (when (ros::time> time-stamp (send msg :header :stamp)) ;;rostime揃え
        (return-from bounding-box-array-cb nil))
      (ros::ros-info "tf")
      (setq obj-pos (scale 0.001 (send (send base-coords :transform base->obj-coords) :worldpos)))
      (setq found-obj t)
      (print obj-pos)
      obj-pos)))

(defun gift-cb (msg)
    (setq *gift-number* (send msg :data)))

;;obj-posを引数にとり，対象物をつかむ
(defun grasp-with-rec (obj-pos)
  (setq target-cds (make-coords :pos (scale 1000 obj-pos)))
  (ros::ros-info "targe-cds 1")
  (ros::ros-info (format nil "~A" target-cds))
  (send target-cds :translate #f(-200 0 50)) ;;z should be 0, but the link is not rigid in gazebo, so 100 is the height offset for end effector.
  (ros::ros-info "targe-cds 2")
  (ros::ros-info (format nil "~A" target-cds))
  (objects (list *fetch* target-cds))

  (send *fetch* :inverse-kinematics target-cds)
  (ros::ros-warn "the grap arm pose is ~A" (send *fetch* :angle-vector))
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)


  (send *fetch* :inverse-kinematics (send target-cds :translate #f(200 0 0)) :rotation-axis :y)
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;; grasp
  ;; still some bug in start-grasp, so we have to directly use :move-gripper
  (send *ri* :start-grasp :effort 70)
  ;;(send *ri* :move-gripper -30 :tm 1500 :wait t)
  (send *ri* :wait-interpolation)

  ;;(send *ri* :go-pos 0 0 60)
  (send *ri* :wait-interpolation)
  (unix::sleep 1)

  (send *ri* :wait-interpolation)
  (unix::sleep 3))

(ros::roseus "boundingboxarray_subscriber")

(unix::sleep 1)
(ros::unsubscribe *gift-name*)

;;基本動作
(defun reset-pose()
    (send *ri* :angle-vector (send *fetch* :reset-pose) 10000))

;;行動
;;行く前
(defun before-go()
    (send *ri* :speak-jp "プレゼントを取りに行きます")
    (reset-pose)
    (send *ri* :wait-interpolation))

;;棚の結構前まで移動
(defun move-to-shelf()
    (send *ri* :speak-jp "棚まで移動します")
    (send *ri* :clear-costmap)
    (send *ri* :move-to (make-coords :pos #f(2001 6500 351) :rpy (float-vector pi/2 0 0)) :frame-id "/map") ;;棚1m前
    (send *ri* :wait-interpolation))

;;棚の前でポーズ
(defun set-grasp-pose()
    (send *ri* :speak-jp "アームが動きます")
    (send *fetch* :angle-vector #f(148.21 46.8197 -40.011 62.3072 -61.0977 76.1935 -100.195 -30.4983 -0.373885 20.5332))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 15000)
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp :effort 70)
    (send *ri* :wait-interpolation))

;;棚の直前まで移動
(defun move-forward()
    (send *ri* :speak-jp "棚に接近します")
    ;;(go-to-spot "/eng2/7f/room73B2-counter-side")
    (send *ri* :go-pos-unsafe 0.55 0 0)
    (setq time-stamp (ros::time-now))
    (send *ri* :wait-interpolation))

;;プレゼントを設定．hsiフィルタのパラメータを変更する
(defun set-gift()
    (print "start set-gift")
    (setq msg (one-shot-subscribe *gift-name* std_msgs::Int16))
    (gift-cb msg)
    (ros::ros-info "msg:~A, number:~A" msg *gift-number*)
    (ros::spin-once)
    (cond
        ((not (boundp '*gift-number*))
            (ros::ros-error "*gift-number* not bound."))
        ((= *gift-number* 1)
            (send *ri* :speak-jp "赤を取ります")
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "h_limit_max" :int 20)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "h_limit_min" :int -20)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "s_limit_max" :int 255)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "s_limit_min" :int 80)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "i_limit_max" :int 255)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "i_limit_min" :int 50))
        ((= *gift-number* 2)
            (send *ri* :speak-jp "青を取ります")
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "h_limit_max" :int -75)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "h_limit_min" :int -96)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "s_limit_max" :int 255)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "s_limit_min" :int 95)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "i_limit_max" :int 255)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "i_limit_min" :int 70))
        ((= *gift-number* 3)
            (send *ri* :speak-jp "緑を取ります")
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "h_limit_max" :int 100)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "h_limit_min" :int 60)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "s_limit_max" :int 255)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "s_limit_min" :int 90)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "i_limit_max" :int 255)
            (ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "i_limit_min" :int 70))))

;;腕を物体まで動かす
(defun catch()
    (setq found-obj nil)
    (ros::ros-info "catch func")
    (until found-obj
        (ros::spin-once)
        (ros::sleep))
    (ros::ros-info "until finished")
    ;;棚との干渉
    (send *ri* :speak-jp "物体をつかみます")
    ;;(setq *shelf* (instance 73b2-cupboard-without-door-object :init))
    (send shelf :translate #f(1100 200 0)) ;;自分の前 mm
    ;;collision-object initialization
    (setq *co* (instance collision-object-publisher :init))
    ;;coをMoveitに送信
    (send *co* :add-object shelf :frame-id "base_link" :relative-pose (make-coords :pos #f(1100 200 0) :rpy #f(4.71 0 0)))
    (grasp-with-rec obj-pos)
    (send *ri* :wait-interpolation)

    (send *fetch* :angle-vector #f(150.109 47.4349 -61.1706 4.45315 -59.0323 124.775 -103.03 -24.8294 0.113294 21.0826))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 6000)
    (send *ri* :wait-interpolation)

    ;;(send *co* :wipe-all)
    )

;;後退
(defun go-back()
    (send *ri* :speak-jp "バックします")
    (send *ri* :go-pos-unsafe -0.7 0 0)
    (send *ri* :wait-interpolation))

(defun atohayoroshiku()
    (send *ri* :speak-jp "かなざわくん，あとはよろしく"))

;;main
(defun main()
    ;;initilalize
    (before-go)
    (send *ri* :wait-interpolation)
    (move-to-shelf)
    (send *ri* :wait-interpolation)
    (set-grasp-pose)
    (send *ri* :wait-interpolation)
    (move-forward)
    (send *ri* :wait-interpolation)
    (set-gift)
    (send *ri* :wait-interpolation)
    (ros::subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray #'bounding-box-array-cb 1)
    (catch)
    (send *ri* :wait-interpolation)
    (go-back)
    (send *ri* :wait-interpolation)
    ;;(atohayoroshiku)
    )

(main)
