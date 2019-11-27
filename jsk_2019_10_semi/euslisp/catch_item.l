;;棚から箱を取ってくる動作

;;load packages
(load "package://fetcheus/fetch-interface.l")
(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
(load "models/73b2-cupboard-without-door-object.l") ;;棚
(ros::load-ros-manifest "jsk_recognition_msgs")

(defvar *topic-name* "/head_camera/depth_registered/boxes")
(defvar *bounding-box-list* nil)

;;callback
(defun bounding-box-array-cb (msg)
  (setq *bounding-box-list* (send msg :boxes)) ;; boxesは、BoundingBoxのArray(Euslispではlist)
  (when *bounding-box-list*
    (let* ((b (elt *bounding-box-list* 0))
           (base->obj-coords (ros::tf-pose->coords (send b :pose)))
           (base-coords (send (send *fetch* :base_link_lk) :copy-worldcoords)))
      (setq obj-pos (scale 0.001 (send (send base-coords :transform base->obj-coords) :worldpos)))
      (setq found-obj t)
      (print obj-pos)
      obj-pos)))


;;基本動作
(defun reset-pose()
    (send *ri* :angle-vector (send *fetch* :reset-pose) 10000))


;;行動
;;行く前
(defun before-go()
    (send *ri* :speak-jp "動き出します")
    (reset-pose)
    (send *ri* :wait-interpolation))

;;棚の結構前まで移動
(defun move-to-shelf()
    (send *ri* :speak-jp "移動します")
    (send *ri* :move-to (make-coords :pos #f(2001 6500 351) :rpy (float-vector pi/2 0 0)) :frame-id "/map") ;;棚1m前
    (send *ri* :wait-interpolation))

;;棚の前でポーズ
(defun set-grasp-pose()
    (send *ri* :speak-jp "アームが動きます")
    (send *fetch* :angle-vector #f(20.2925 46.7685 -39.9745 62.3554 -61.1541 76.2204 -100.1 -30.3983 -5.81212 27.5645))
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
    (send *ri* :stop-grasp)
    (send *ri* :wait-interpolation))

;;棚の直前まで移動
(defun move-forward()
    (send *ri* :speak-jp "棚の直前まで移動します")
    ;;(go-to-spot "/eng2/7f/room73B2-counter-side")
    (send *ri* :go-pos-unsafe 0.6 0 0)
    (send *ri* :wait-interpolation))

;;腕を物体まで動かす
(defun move-arm-to-target()
    ;;棚との干渉
    (setq *shelf* (instance 73b2-cupboard-without-door-object :init))
    (send *shelf* :trancelate #f(500 0 0)) ;;自分の前500mm
    (setq *co* (instance collision-object-publisher :init))
    (send *co* :add-object *shelf* :frame-id "base_link" :relative-pose (make-coords :pos #f(500 0 0)))
    (setq found-obj nil)
    (send *)
    )



;;つかむ
(defun catch()
    (send *ri* :speak-jp "箱を掴みます")
    (send *fetch* :angle-vector #f()) ;;腕を下ろす
    (send *ri* :angle-vector (send *fetch* :angle-vector) 4000)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp) ;;つかむ

    (send *fetch* :angle-vector #f()) ;;腕を上げる
    (send *ri* :angle-vector (send *fetch* :angle-vector) 4000) ;; 
    (send *ri* :wait-interpolation) 


;;main
(defun main()
    (before-go)
    (move-to-shelf))
