(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(load "models/foldable-desk-object.l")
(ros::load-ros-manifest "jsk_recognition_msgs")

(defvar *topic-name* "/head_camera/depth_registered/boxes")
(defvar *bounding-box-list* nil)

;;コールバック関数
(defun bounding-box-array-cb (msg)
  (print 2)
  (setq *bounding-box-list* (send msg :boxes)) ;; boxesは、BoundingBoxのArray(Euslispではlist)
  (when *bounding-box-list*
    (let* ((b (elt *bounding-box-list* 0))
           (base->obj-coords (ros::tf-pose->coords (send b :pose)))
           (base-coords (send (send *fetch* :base_link_lk) :copy-worldcoords)))
      (setq obj-pos (scale 0.001 (send (send base-coords :transform base->obj-coords) :worldpos)))
      (setq found-obj t)
      (print obj-pos)
      obj-pos)))

;;認識した位置のものを上からつかむ
(defun grasp-with-rec (obj-pos)
  (setq target-cds (make-coords :pos (scale 1000 obj-pos)))
  (send target-cds :translate #f(0 0 200))
  (send target-cds :rotate (deg2rad 90) :y);;上からつかむために目標の座標を回転
  (objects (list *fetch* target-cds))

  ;;物体の少し上でikを解く
  (print target-cds)
  (send *fetch* :inverse-kinematics target-cds :rotation-axis t)
  (ros::ros-warn "the grap arm pose is ~A" (send *fetch* :angle-vector))
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;;直動リンクのtorsoを下げる
  (send *fetch* :torso :waist-z :joint-angle (- (send *fetch* :torso :waist-z :joint-angle ) 170))
  (send *ri* :angle-vector (send *fetch* :angle-vector))
  (send *ri* :wait-interpolation)
  ;;(send *fetch* :inverse-kinematics (send target-cds :translate #f(200 0 0)) :rotation-axis t)
  ;;(send *ri* :angle-vector  (send *fetch* :angle-vector) 3000)
  ;;(send *ri* :wait-interpolation)

  ;; grasp
  ;; still some bug in start-grasp, so we have to directly use :move-gripper
  (send *ri* :start-grasp :effort 70.0)
  ;;(send *ri* :move-gripper -30 :tm 1500 :wait t)
  (send *ri* :wait-interpolation))

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


  (print 'joudan)
  ;;横バーション
  ;;(send *fetch* :angle-vector #f(280.817 43.6337 24.4568 68.7891 -61.2735 32.7315 -81.5184 -93.8235 -3.27428 28.9048))
  ;;上段
  (send *fetch* :angle-vector #f(280.505 46.8227 -39.9922 62.3577 -61.1469 76.2949 -100.012 -30.4152 -5.81212 27.5645))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (unix::sleep 3)

  ;;(print 3)
  (setq found-obj nil)
  (until found-obj
    (print 4)
    ;;(x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
    (ros::spin-once)
    (ros::sleep))

  ;;机との干渉
  (setq *table* (instance foldable-desk-object :init))
  (send *table* :translate #f(800 0 0))
  (setq *co* (instance collision-object-publisher :init))
  (send *co* :add-object *table* :frame-id "base_link" :relative-pose (make-coords :pos #f(800 0 0) ))

   ;;オブジェクトを見る
  (send *fetch* :head :look-at (scale 1000 obj-pos))
  (send *ri* :angle-vector (send *fetch* :angle-vector))
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :effort 70.0)

  ;;(send *co* :wipe-all)
  (setq found-obj nil)
  (until found-obj
    (print 5)
    ;;(x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
    (ros::spin-once)
    (ros::sleep))

  ;;物をつかむ
  (grasp-with-rec obj-pos)

  ;;(send *ri* :speak-jp "つかみました！")
  (unix::sleep 2)

  ;;上段
  (print 'joudan)
  (send *fetch* :angle-vector #f(289.118 46.8636 -39.9231 62.2852 -61.2295 76.3473 -100.085 -30.4104 -2.84375 28.2456))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *co* :wipe-all)
  )

(main)
