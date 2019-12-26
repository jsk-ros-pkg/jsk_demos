(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(load "models/foldable-desk-object.l")
(ros::load-ros-manifest "geometry_msgs")


(defvar *topic-name* "/object_pose")
(defvar *object-pose* nil)

;;コールバック関数
(defun object-pose-cb (msg)
  (print 2)
	(setq *msg* msg)
  ;; (setq *bounding-box-list* (send msg :pose)) ;; boxesは、BoundingBoxのArray(Euslispではlist)
  (when msg
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
    (let* ((b (send msg :pose))
           (cam->obj-coords (ros::tf-pose->coords b))
           (cam-coords (send (send *fetch* :head_camera_rgb_optical_frame_lk) :copy-worldcoords)))
      (setq obj-pos (scale 0.001 (send (send cam-coords :transform cam->obj-coords) :worldpos)))
      (setq found-obj t)
      (print obj-pos)
      obj-pos))
  )

(defun sox-with-rec (obj-pos)
  (setq target-cds (make-coords :pos (scale 1000 obj-pos)))
  (send target-cds :translate #f(30 -100 200))
  ;;(send target-cds :rotate (deg2rad 90) :y :world);;上からつかむために目標の座標を回転
  (send target-cds :rotate (deg2rad -90) :z :world);;上からつかむために目標の座標を回
  (objects (list *fetch* target-cds))

  ;;物体の少し上でikを解く
  (print target-cds)
  (send *fetch* :inverse-kinematics target-cds :rotation-axis t)
  (ros::ros-warn "the grap arm pose is ~A" (send *fetch* :angle-vector))
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;;手を離す位置に移動する。
  (send *fetch* :inverse-kinematics (send target-cds :translate #f(0 0 -100) :world) :rotation-axis :t)
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;; relase
  (send *ri* :stop-grasp)
  (send *ri* :wait-interpolation)

  (send *fetch* :inverse-kinematics (send target-cds :translate #f(0 200 0) :world) :rotation-axis :t)
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  )

(defun main nil
    ;; ros initialization
  (ros::roseus "object_pose_subscriber")
  (ros::subscribe *topic-name* geometry_msgs::PoseStamped #'object-pose-cb 1)
  ;; eus initialization
  (if (not (boundp '*fetch*))
      (progn (load "package://fetcheus/fetch-interface.l")
	     (fetch-init)))
  (objects (list *fetch*))
  (setq found-obj nil)
  (setq obj-pos #f(0 0 0))


  ;;物を掴ませてもらうポーズ(本当は袋から取り出す)
  (send *fetch* :angle-vector #f(5.52373 52.5326 3.4729 77.8638 -59.6255 10.2535 -78.2225 74.2234 0.022649 2.23))
  (send *ri* :angle-vector (send *fetch* :angle-vector))
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :effort 70.0 :wait t)
  (unix:sleep 3)
  (send *ri* :start-grasp :effort 70.0 :wait t)

  ;;ポーズを変える
  ;;上段
  (print 'joudan)
  (send *fetch* :angle-vector #f(289.118 46.8636 -39.9231 62.2852 -61.2295 76.3473 -100.085 -30.4104 -2.84375 28.2456))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (unix::sleep 2)
  ;;移動とかをする

  ;;コールバックを呼ぶ
  (setq found-obj nil)
  (until found-obj
	 (print 4)
  ;;(x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
	 (ros::spin-once)
	 (ros::sleep))

  ;;壁との干渉
  ; (setq *wall* (make-cube 100 4000 5000))
  ; (send *wall* :locate #f(0 0 2500))
  ;;(setq *co* (instance collision-object-publisher :init))
  ;;(send *co* :add-object *table* :frame-id "base_link" :relative-pose (make-coords :pos #f(800 0 0) ))

  (sox-with-rec obj-pos)
  )

(main)
