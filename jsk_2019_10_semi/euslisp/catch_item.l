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
(defvar *bounding-box-list* nil)

(fetch-init)
(objects (list *fetch*))

;;callback
(defun bounding-box-array-cb (msg)
  (ros::ros-info "cb called")
  (setq *bounding-box-list* (send msg :boxes)) ;; boxesは、BoundingBoxのArray(Euslispではlist)
  (ros::ros-info "set box-list")
  (when *bounding-box-list*
    (let* ((b (elt *bounding-box-list* 0))
           (base->obj-coords (ros::tf-pose->coords (send b :pose)))
           (base-coords (send (send *fetch* :base_link_lk) :copy-worldcoords)))
      (ros::ros-info "tf")
      (setq obj-pos (scale 0.001 (send (send base-coords :transform base->obj-coords) :worldpos)))
      (setq found-obj t)

      ;;(print found-obj)
      (print obj-pos)
      obj-pos)))

;;obj-posを引数にとり，対象物をつかむ
(defun grasp-with-rec (obj-pos)
  (setq target-cds (make-coords :pos (scale 1000 obj-pos)))
  (send target-cds :translate #f(-200 0 50)) ;;z should be 0, but the link is not rigid in gazebo, so 100 is the height offset for end effector.
  (objects (list *fetch* target-cds))

  (send *fetch* :inverse-kinematics target-cds :rotation-axis :y)
  (ros::ros-warn "the grap arm pose is ~A" (send *fetch* :angle-vector))
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)


  (send *fetch* :inverse-kinematics (send target-cds :translate #f(200 0 0)) :rotation-axis :y)
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;; grasp
  ;; still some bug in start-grasp, so we have to directly use :move-gripper
  (send *ri* :start-grasp)
  ;;(send *ri* :move-gripper -30 :tm 1500 :wait t)
  (send *ri* :wait-interpolation)

  ;;(send *ri* :go-pos 0 0 60)
  (send *ri* :wait-interpolation)
  (unix::sleep 1)

  (send *ri* :wait-interpolation)
  (unix::sleep 3))

(ros::roseus "boundingboxarray_subscriber")
(ros::subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray #'bounding-box-array-cb 1)

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
    (send *ri* :move-to (make-coords :pos #f(2001 6500 351) :rpy (float-vector pi/2 0 0)) :frame-id "/map") ;;棚1m前
    (send *ri* :wait-interpolation))

;;棚の前でポーズ
(defun set-grasp-pose()
    (send *ri* :speak-jp "アームが動きます")
    (send *fetch* :angle-vector #f(39.3115 46.7395 -40.022 62.3556 -61.1188 76.2675 -100.145 -30.4329 -5.81212 27.5645))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 15000)
    (send *ri* :stop-grasp)
    (send *ri* :wait-interpolation))

;;棚の直前まで移動
(defun move-forward()
    (send *ri* :speak-jp "棚に接近します")
    ;;(go-to-spot "/eng2/7f/room73B2-counter-side")
    (send *ri* :go-pos-unsafe 0.5 0 0)
    (send *ri* :wait-interpolation))

;;腕を物体まで動かす
(defun catch()
    (setq found-obj nil)
    (ros::ros-info "catch func")
    (until found-obj
        (ros::spin-once)
        (ros::sleep))
    (ros::ros-info "until finished")
    (setq obj-pos #f(0 0 0))
    ;;棚との干渉
    (send *ri* :speak-jp "物体をつかみます")
    ;;(setq *shelf* (instance 73b2-cupboard-without-door-object :init))
    (send shelf :translate #f(800 200 0)) ;;自分の前 mm
    ;;collision-object initialization
    (setq *co* (instance collision-object-publisher :init))
    ;;coをMoveitに送信
    (send *co* :add-object shelf :frame-id "base_link" :relative-pose (make-coords :pos #f(800 200 0) :rpy #f(4.71 0 0))) ;;
    (grasp-with-rec obj-pos)
    (send *co* :wipe-all))

;;後退
(defun go-back()
    (send *ri* :speak-jp "バックします")
    (send *ri* :go-pos -0.7 0 0)
    (send *ri* :wait-interpolation))

(defun atohayoroshiku()
    (send *ri* :speak-jp "かなざわくんあとはよろしく"))

;;main
(defun main()
    ;;initilalize

    (before-go)
    (move-to-shelf)
    (set-grasp-pose)
    (move-forward)
    ;;(catch)
    (go-back)
    (atohayoroshiku))

;;(main)