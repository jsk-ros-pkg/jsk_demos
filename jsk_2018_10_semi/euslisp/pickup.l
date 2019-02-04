;;(ros::load-ros-manifest "jsk_pcl_ros")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "fetch_driver_msgs")

;; クラスタリング結果であるBoundingBoxのtopic名
(defvar *bounding-box-topic* "/delivery_box_pickup/cluster_point_indices_decomposer_align_boxes_with_plane/boxes")
(defvar *bounding-box-list* nil)
(defvar *bx-list* nil)
;;(setq *co* (instance collision-object-publisher :init))
;;(setq *table* (make-cube 800 800 800))
;; ros::initする
(ros::roseus "pick-up-box")

(setq look-down-pose #f(386.15 75.6359 80.1929 -11.4118 98.5702 0.060285 95.1322 -0.060071 -0.895055 42.9474));;見下ろすポーズ

(defun goto-kitchenside nil
  (send *ri* :move-to (send (send (send *eng2-scene* :spot "coe-spot") :copy-worldcoords)
			       :transform (make-coords :pos #f(0 0 0) :rpy (float-vector -0.5 0 0))))
  (send *fetch* :move-to (send (send (send *eng2-scene* :spot "coe-spot") :copy-worldcoords)
			       :transform (make-coords :pos #f(0 0 0) :rpy (float-vector -0.5 0 0))) :world)
  (send *ri* :angle-vector look-down-pose 5000)
  (send *fetch* :angle-vector look-down-pose)
  (send *irtviewer* :look-all *fetch*)
  )


(defun detect-box-cb (msg)
  (setq *bounding-box-list* (send msg :boxes)) ;; boxesは、BoundingBoxのArray(Euslispではlist)
  (setq *bx-list* nil)
  ;; BoundingBoxがあれば表示する
  (when *bounding-box-list*
    (send *irtviewer* :draw-objects :flush nil)
    (mapcar #'(lambda (b)
                ;; BoundingBoxは、dimsntion(直方体の幅・奥行き・高さ)をもつ
                (let* ((dims (ros::tf-point->pos (send b :dimensions)))
                       (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2)))
                       ;; (1) BoundingBoxのカメラ相対の座標系は、geometry_msgs/Poseという型で得られるので、Euslispのcoordsに変換する
                       (cam->obj-coords (ros::tf-pose->coords (send b :pose)))
                       ;; (2) *dxl-armed-turtlebot*モデルがカメラの座標系をもってるので、取得する
                       (cam-coords (send (send *fetch* :head_camera_rgb_optical_frame_lk) :copy-worldcoords)))
                  ;; (3) Euslisp内部でのworld座標系の値にして、そこにmake-cubeの箱を設置する
                  (send bx :newcoords (send cam-coords :transform cam->obj-coords))
                  (send bx :worldcoords)
		  (setq *bx-list* (append *bx-list* (list bx)))
                  (send bx :draw-on :flush nil :color #f(1 0 0)) ;; 描画
		  bx))
            *bounding-box-list*)
    (format t "# of box detecetd : ~A " (length *bx-list*))
    (send *irtviewer* :viewer :viewsurface :flush)
    )
  )

(defun try-to-pickup nil
  ;;箱を取ることを試みる
  (progn
    (print "picking up...")
    (unix:sleep 1)
    (setq gripper-state (one-shot-subscribe "gripper_state" fetch_driver_msgs::GripperState))
    ;;gripper-state の状態をgripper-stateに入れる
    (setq gripper-positon (elt (send gripper-state :joints) 0) :position)
    ;;gripper-position の状態をgripper-stateに入れる
    (format t "gripper position :~A ~%" (send (elt b 0) :position))
    (if (<= gripper-position 100)
	(setq pickup-status 1)
      (setq pickup-status 0)
      ) 
    )
  )

(defun check-address nil
  (print "cheking address...")
  )

(defun store-box nil
  (print "storing box...")
  )

(defun publish-pickup-done nil
  (print "done")
  )

(ros::subscribe *bounding-box-topic* jsk_recognition_msgs::BoundingBoxArray #'detect-box-cb 1)

(setq pickup-status 1)

(do-until-key
 (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
 (if *bx-list*
     (progn
       (setq tmp *bx-list*)
       (try-to-pickup)
       (if (= pickup-status 1)
	   (progn
	     (check-address)
	     (store-box)
	     ))
       )
   (publish-pickup-done))
   (ros::spin-once)
   (ros::sleep)
 )
