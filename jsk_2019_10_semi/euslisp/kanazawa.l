;;中継をする関数
(defun chukei nil
  ;;ポーズの中継
  (send *fetch* :angle-vector #f(300 47.4349 -61.1706 4.45315 -59.0323 124.775 -103.03 -24.8294 0.113294 21.0826))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 20000)
  (send *ri* :wait-interpolation)

  (send *fetch* :angle-vector #f(300 72.5277 80.2673 -13.5644 98.9291 -21.1234 94.8342 -1.38452 0.54382 27.147))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 20000 :arm-controller 0 :move-arm :rarm)
  (send *ri* :wait-interpolation)

  ;;移動をするポーズ
  (send *fetch* :angle-vector #f(53.9173 72.5277 80.2673 -13.5644 98.9291 -21.1234 94.8342 -1.38452 0.54382 27.147))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (setq *co* (instance collision-object-publisher :init))
  (send *co* :wipe-all)

  (send *ri* :speak-jp "いいコのみんなにはプレゼントをあげるよ！！！")
  (go-to-spot "/eng2/7f/room73B2-door-front" (make-coords :pos #f(200 0 0)))
  (go-to-spot "/eng2/7f/room73A3-front"))

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
      (if (and (< (elt obj-pos 0) 0.7) (< (elt obj-pos 1) 0.25))
        (progn (setq found-obj t)
               (print obj-pos)))
      obj-pos))
  )

(defun sox-with-rec (obj-pos)
  (setq target-cds (make-coords :pos (scale 1000 obj-pos)))
  (send target-cds :translate #f(30 -40 200)) ;;もともと30 -100 200
  ;;(send target-cds :rotate (deg2rad 90) :y :world);;上からつかむために目標の座標を回転
  (send target-cds :rotate (deg2rad -90) :z :world);;上からつかむために目標の座標を回
  (objects (list *fetch* target-cds))

  ;;物体の少し上でikを解く
  (print target-cds)

  ;; 少し上に移動する
  (send *fetch* :inverse-kinematics target-cds :rotation-axis t)
  (ros::ros-warn "the grap arm pose is ~A" (send *fetch* :angle-vector))
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 6000)
  (send *ri* :wait-interpolation)

  ;;手を離す位置に移動する。
  (send *fetch* :inverse-kinematics (send target-cds :translate #f(0 0 -100) :world) :rotation-axis :t)
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 6000)
  (send *ri* :wait-interpolation)

  ;; relase
  (send *ri* :stop-grasp :effort 70.0 )
  (send *ri* :wait-interpolation)

  (send *fetch* :inverse-kinematics (send target-cds :translate #f(0 100 0) :world) :rotation-axis :t)
  (send *ri* :angle-vector  (send *fetch* :angle-vector) 6000)
  (send *ri* :wait-interpolation)

  ;;予備ポーズ
  (print 'yobi)
  (send *fetch* :angle-vector #f(345.972 59.1877 -12.9058 -167.611 95.3699 34.8689 64.959 13.4483 -1.64847 45.2379))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 6000)
  (send *ri* :wait-interpolation)

  )

(defun door-socks-demo nil
  ;;main
  (ros::roseus "object_pose_subscriber")
  (ros::subscribe *topic-name* geometry_msgs::PoseStamped #'object-pose-cb 1)
  (setq found-obj nil)
  (setq obj-pos #f(0 0 0))

  ;;壁との干渉計算
  (setq *wall* (make-cube 100 4000 5000))
  (send *wall* :locate #f(50 0 2500))
  (setq *co* (instance collision-object-publisher :init))
  (send *co* :add-object *wall* :frame-id "map" :relative-pose (make-coords :pos #f(-2200.0 6300.0 0.0) ))

  (unix::sleep 2)
  ;;予備ポーズ
  (print 'yobi)
  (send *fetch* :angle-vector #f(345.972 59.1877 -12.9058 -167.611 95.3699 34.8689 64.959 13.4483 -1.64847 45.2379))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 6000)
  (send *ri* :wait-interpolation)

  ;;前に進む
  (send *ri* :go-pos-unsafe 0.14 0 0)
  (send *ri* :wait-interpolation)

  ;下を向く
  ; (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  ; (send *fetch* :head :look-at #f(700 0 1100))
  ; (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  ; (send *ri* :wait-interpolation)

  (unix:sleep 3)
  ;;コールバックを呼ぶ
  (setq found-obj nil)
  (until found-obj
   (print 4)
  ;;(x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
   (ros::spin-once)
   (ros::sleep))

  (send *fetch* :head :look-at (scale 1000 obj-pos))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (sox-with-rec obj-pos)


  ;;回転
  (send *ri* :go-pos-unsafe 0 0 90)
  (send *ri* :wait-interpolation)
  (send *ri* :go-pos-unsafe 0 0 90)
  (send *ri* :wait-interpolation)

  ;;リセットポーズ
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 6000)
  (send *ri* :wait-interpolation)

  (send *ri* :speak-jp "メリークリスマス")
  ;;(send *ri* :speak "merry christmas from jsk 2019 zemi")

  (send *co* :wipe-all))

(defun kanazawa nil
  (load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
  (load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
  (ros::load-ros-manifest "geometry_msgs")

  (defvar *topic-name*)
  (setq *topic-name* "/object_pose")
  (defvar *object-pose*)
  (setq *object-pose* nil)

  (if (not (boundp '*fetch*))
      (progn (load "package://fetcheus/fetch-interface.l")
	     (fetch-init)
	      (objects (list *fetch*))))

  (chukei)
  (door-socks-demo))
