(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(load "models/foldable-desk-object.l")
(ros::load-ros-manifest "jsk_recognition_msgs")

;;変数を変更
(defvar *topic-name* "/head_camera/depth_registered/boxes")
(defvar *bounding-box-list* nil)

;;コールバック関数、オブジェクトを発見する
(defun bounding-box-array-cb (msg)
  (print 2)
  (setq *bounding-box-list* (send msg :boxes)) ;; boxesは、BoundingBoxのArray(Euslispではlist)
  (when *bounding-box-list*
    (let* ((b (elt *bounding-box-list* 0))
           (cam->obj-coords (ros::tf-pose->coords (send b :pose)))
           (cam-coords (send (send *fetch* :head_camera_depth_optical_frame_lk) :copy-worldcoords)))
      (setq obj-pos (scale 0.001 (send (send cam-coords :transform cam->obj-coords) :worldpos)))
      (setq found-obj t)
      (print obj-pos)
      obj-pos)))

;;認識してつかむ
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

  (send *ri* :go-pos 0 0 60)
  (send *ri* :wait-interpolation)
  (unix::sleep 1)

  (send *ri* :stop-grasp)
  (send *ri* :wait-interpolation)
  (unix::sleep 3)
  )

;;メイン
(defun main nil
  ;; ros initialization
  (ros::roseus "boundingboxarray_subscriber")
  (ros::subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray #'bounding-box-array-cb 1)

  ;; eus initialization
  (if (not (boundp '*fetch*))
      (progn (load "package://fetcheus/fetch-interface.l")
             (fetch-init)))
  (objects (list *fetch*))
  (setq found-obj nil)
  (setq obj-pos #f(0 0 0))

  ;;リセットポーズ
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)


  ;;物を認識するまでサブスクライブ
  (setq found-obj nil)
  (until found-obj
     (print 4)
     ;;(x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
     (ros::spin-once)
     (ros::sleep))

  ;;２次元の位置を取得
  (setq 2d-pos (float-vector (elt obj-pos 0) (elt obj-pos 1)))

  ;;80cm手前まで移動
  (send *ri* :go-pos
    (elt (scale (/ (- (norm 2d-pos) 0.8) (norm 2d-pos)) 2d-pos) 0)
    (elt (scale (/ (- (norm 2d-pos) 0.8) (norm 2d-pos)) 2d-pos) 1)
    (rad2deg (atan  (elt obj-pos 1) (elt obj-pos 0))))

  ;;机との干渉を計算
  (setq *table* (instance foldable-desk-object :init))
  (send *table* :translate #f(800 0 0))
  (setq *co* (instance collision-object-publisher :init))
  ;;これでコリジョンオブジェクトをMoveitに送る
  (send *co* :add-object *table* :frame-id "base_link" :relative-pose (make-coords :pos #f(800 0 0 ))

  ;;上段
  (print 'joudan)
  (send *fetch* :angle-vector #f(298.136 43.1722 -39.9011 62.2852 -56.835 81.379 -104.018 -40.364 0.305897 13.8975))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;;Moveitのコリジョンオブジェクトを取り消す
  (send *co* :wipe-all)

  ;;手を開く
  (send *ri* :stop-grasp)

  ;;もう一度サブスクライブして物体位置確認
  (setq found-obj nil)
  (until found-obj
     (print 5)
     ;;(x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
     (ros::spin-once)
     (ros::sleep))

#|

  ;;0.6m手前まで近づく
  (setq 2d-pos (float-vector (elt obj-pos 0) (elt obj-pos 1)))
  (send *ri* :go-pos-unsafe
    (elt (scale (/ (- (norm 2d-pos) 0.6) (norm 2d-pos)) 2d-pos) 0)
    (elt (scale (/ (- (norm 2d-pos) 0.6) (norm 2d-pos)) 2d-pos) 1)
    (rad2deg (atan  (elt obj-pos 1) (elt obj-pos 0))))

|#

  ;;物体をつかむ
  (grasp-with-rec obj-pos)

  )

(main)


