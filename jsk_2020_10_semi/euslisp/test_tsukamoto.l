#!/usr/bin/env roseus

;; load robot and scene model
(require "package://fetcheus/fetch-utils.l")
(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(unless (boundp '*scene*)
  (setq *scene* (make-eng2-7f-scene)))
(unless (boundp '*ri*)
  (fetch-init)) ;; fetch-init instantiate *fetch* and *ri*

(setq *area* 0)

;; show in irtviewer
(objects (list *fetch* *scene*))
;; show in "fetch Kinematics Simulator"
(send *ri* :objects (send *scene* :objects))

;; dockの前まで移動する
(defun move-dock-front ()
  (setq *dock-front*
	(send (send *scene* :spot "/eng2/7f/room73B2-fetch-dock-front")
		      :copy-worldcoords))
  (send *fetch* :move-to *dock-front* :world)
  (send *ri* :move-to *dock-front*))

;; fetchを見やすくする
(defun view-fetch ()
    (send *irtviewer* :look-all
	  (send (geo::make-bounding-box
		 (flatten (send-all (send *fetch* :bodies) :vertices)))
		:grow 2.0))
    )
;; 73A2のまえまで移動する
(defun move-73-A2-front ()
  (setq *room73A2-front* (send (send *scene* :spot "/eng2/7f/room73A2-front") :copy-worldcoords))
  (send *fetch* :move-to *room73A2-front* :world)
  (send *ri* :move-to *room73A2-front*)
  )
;;(send *fetch* :locate #f(300 0 0))

;;(send *ri* :move-to *room73A2-front*)
#|
(setq *room73A3-front* (send (send *scene* :spot "/eng2/7f/room73A3-front") :copy-worldcoords))
(send *fetch* :move-to *room73A3-front* :world)

|#

;;reset-poseする
(defun reset-pose (&optional (tm 3000))
    (send *fetch* :reset-pose)
    (send *ri* :angle-vector (send *fetch* :angle-vector) tm)
    (send *ri* :wait-interpolation))

(defun pose-of-give ()
  (send *ri* :start-grasp :effort 60) ;; fetchの手は掴んだ状態
  (unix:sleep 1) ;; 掴んだ状態を作る
  ;;(send *fetch* :torso_lift_joint :joint-angle 60)
  ;;(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  ;;(send *fetch* :stop-grasp)

  (setq hand-coord (send (send *fetch* :copy-worldcoords) :transform
			 (make-coords :pos #f(600 100 1000) :rpy (float-vector 0 0 pi/2))))
  
  #| fetchの右手をものを受け取る位置に持っていく |#
  (send *fetch* :rarm :inverse-kinematics
	(send (send *fetch* :copy-worldcoords) :transform
	      (make-coords :pos #f(600 100 1000) :rpy (float-vector 0 0 pi/2))
	      ) :debug-view t)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation t) ;; 前の動作が終わるまで少し待つ
  ;;(send *fetch* :wrist_roll_joint :joint-angle 0)
  ;;(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation t)
  (send *ri* :stop-grasp)
  (unix::sleep 3)
  (send *ri* :start-grasp :effort 60)

  
  #|
  (send *ri* :wait-interpolation t)
  (send *ri* :start-grasp)
  (send *ri* :wait-interpolation t)
  |#
  #|
  (send *fetch* :torso_lift_joint :joint-angle 60)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation t)
  |#
  ;;(send *ri* :angle-vector (send *fetch* :reset-pose) 5000)
  )
  ;;(send *fetch* :rarm :angle-vector)
  
#| 頭の上に手を持ってくる |#
(defun raise-hand ()
  (send *fetch* :angle-vector #f(5.0000 88.0000 -42.0000 0 -60 1 -77.0000 7.000 0 2.0000))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  )

#| 手を前に突き出す感じ |#
(defun catch ()
  (send *fetch* :angle-vector #f(5 52 3 77 -59 10 -78 74 0 2))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  )

#| 顔を検知してそこに描かれた長方形の面積を求める |#
(defclass face_area
  :slots (*rect* *area*)
  )
(defmethod face_area
  (:init ()
	 (ros::roseus "face_recognized_client")
	 (ros::subscribe "edgetpu_face_detector/output/rects"
			 jsk_recognition_msgs::RectArray #'send self :cb)
	 (setq *rect* nil)
	 (ros::ros-info "init")
	 self)
  (:cb (msg)
       (setq *rect* (send msg :rects))
       )
  (:loop ()
	 (ros::rate 10)
	 (while (ros::ok)
	   (if *rect*
	       (progn
		 (setq w (send (elt *rect* 0) :width))
		 (setq h (send (elt *rect* 0) :height))
		 (setq *area* (* w h)) ;; 大域変数
		 (ros::ros-info "area : ~A" *area*)
		 )
	     )
	     (ros::spin-once)
	     (ros::sleep)
	    )
	 (exit)
	 )
  )

;; 顔を見つけたときの挙動
(defun face-detect ()
  (if (> *area* 40000)
      (pose-of-give)
    )
  )

#| 相対座標の取得 |#
(defun make-relative-coord (x y z roll pitch yaw)
  (setq descartes (float-vector x y z))
  (setq euler (float-vector roll pitch yaw))
  (setq relative-coord (send (send *fetch* :copy-worldcoords) :transform
			     (make-coords :pos descartes :rpy euler))
       )
  )

#| 相対位置に右手を動かす |#
(defun relative-move-hand (x y z roll pitch yaw)
  (let coords
       (setq coords (make-relative-coord x y z roll pitch yaw))
       (send *fetch* :rarm :inverse-kinematics coords :debug-view t)
       (send *ri* :angle-vector (send *fetch* :angle-vector)))
  )

#| ものを受け取る |#
(defun receive-flow ()
  (send *ri* :stop-grasp :effort 60)
  (unix::sleep 3)
  (relative-move-hand 600 100 1000 0 0 pi/2)
  (send *ri* :wait-interpolation nil)
  (unix::sleep 5)
  (send *ri* :start-grasp :effort 60)
  (unix::sleep 5)
  (relative-move-hand 500 300 1000 0 0 pi/2)
  (send *ri* :wait-interpolation nil)
  (relative-move-hand 400 400 1000 0 0 pi/2)
  (send *ri* :wait-interpolation nil)
  (relative-move-hand 300 400 1200 0 0 pi/2)
  (send *ri* :wait-interpolation nil)
  (relative-move-hand 300 200 1200 0 0 pi/2)
  )

#| ものを渡す |#
(defun give-flow ()
  ;; 相対位置#f(300 200 1200)にいることを想定
  (relative-move-hand 600 200 1200 0 0 pi/2)
  (send *ri* :wait-interpolation nil)
  (relative-move-hand 600 0 1200 0 0 pi/2)
  (unix::sleep 3)
  (send *fetch* :stop-grasp)
  (send *ri* :stop-grasp)
  (unix::sleep 3)
  )
