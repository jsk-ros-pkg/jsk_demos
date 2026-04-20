#!/usr/bin/env roseus

;; load robot and scene model
(require "package://fetcheus/fetch-utils.l")
(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")

(load "utils_tsukamoto.l")

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(unless (boundp '*scene*)
  (setq *scene* (make-eng2-7f-scene)))
(unless (boundp '*ri*)
  (fetch-init)) ;; fetch-init instantiate *fetch* and *ri*



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
(setq *area* 0)
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
		 (setq x (send (elt *rect* 0) :x))
		 (setq y (send (elt *rect* 0) :y))
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
  (:loop-head-move ()
	 (ros::rate 0.5)
	 (while (ros::ok)
	   (if *rect*
	       (progn
		 (setq x (send (elt *rect* 0) :x))
		 (setq y (send (elt *rect* 0) :y))
		 (ros::ros-info "area : ~A" x)
		 (if (> x 250)
		     (send *fetch* :head :angle-vector #f(30 0))
		     (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
		     )
		 (if (< x 100)
		     (send *fetch* :head :angle-vector #f(-30 0))
		     (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
		     )
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
(defun make-relative-coord (x y z &optional (roll 0) (pitch 0) (yaw 0))
  (setq descartes (float-vector x y z))
  (setq euler (float-vector roll pitch yaw))
  (setq relative-coord (send (send *fetch* :copy-worldcoords) :transform
			     (make-coords :pos descartes :rpy euler))
       )
  )

#| 相対位置に右手を動かす |#
(defun relative-move-hand (x y z &optional (roll 0) (pitch 0) (yaw 0) (ut t))
  (let coords
       (setq coords (make-relative-coord x y z roll pitch yaw))
       (send *fetch* :rarm :inverse-kinematics coords :debug-view t)
       (send *ri* :angle-vector (send *fetch* :angle-vector) 3000 :use-torso t)
       (send *ri* :wait-interpolation nil))
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
  
  (relative-move-hand 600 0 1200 0 0 pi/2)
  (unix::sleep 3)
  (send *ri* :speak-jp "うーばーいーつです")
  
  (setq gripper-pos (* 0.001 2 (vmean (send *ri* :robot :rarm :gripper :angle-vector))))
  ;;(while (< gripper-pos 0.0001)
  (send *fetch* :stop-grasp)
  (send *ri* :stop-grasp)
  (unix::sleep 3)
  (send *fetch* :start-grasp)
  (send *ri* :start-grasp :effort 100) ;; effort 100にするのがミソ
  ;;  )
  (relative-move-hand 500 -300 1000 (/ -pi 2) 0 pi/2)
  (relative-move-hand 400 -300 1000 (/ -pi 2) 0 pi/2)
  (relative-move-hand -200 -400 750 -pi 0 pi/2)
  )

;;(relative-move-hand 300 -400 1000 0 0 0)

;;(relative-move-hand 600 0 600 0 0 pi/2)
;;(relative-move-hand 300 -400 200 0 pi/2 0)

;;(relative-move-hand 100 -400 200 0 pi/2 0)
;;(relative-move-hand -100 -400 200 0 pi/2 0)

;;(relative-move-hand 0 -500 550 pi 0 pi/2)

#|
#| 足元に置かれたUberEatsの蓋を開ける |#
(defun box-open ()
  ;;腰を上げてからのほうが手がリュックに当たらない
  (send *fetch* :torso_lift_joint :joint-angle 100)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation nil)
  
  (relative-move-hand 100 -500 550 pi 0 pi/2)
  (send *ri* :wait-interpolation nil)
  (unix::sleep 1) ;; sleepさせないと挙動がおかしい

  (setq shoulder-y-angle-10 (- (send *fetch* :rarm-shoulder-y :joint-angle) 10))
  (send *fetch* :rarm-shoulder-y :joint-angle shoulder-y-angle-10)
  ;;(send *fetch* :rarm :angle-vector #f(-60.9601 55.1198 -152.558 64.3187 71.9964 114.228 173.586)) ;; 元のものから１つめを-20度した
  
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :use-torso nil)
  (send *ri* :wait-interpolation nil)
  ;; ここから蓋開け動作
  (relative-move-hand 0 -500 650 pi 0 pi/2)
  (send *ri* :wait-interpolation nil)
  (relative-move-hand 0 -500 700 pi 0 pi/2)
  (send *ri* :wait-interpolation nil)
  (relative-move-hand 100 -500 800 pi 0 pi/2)
  (send *ri* :wait-interpolation nil)
  (relative-move-hand 280 -500 870 pi 0 pi/2)
  (send *ri* :wait-interpolation nil)
  #|
  (relative-move-hand 280 -550 870 pi 0 pi/2)
  (send *ri* :wait-interpolation nil)
  |#
  ;; 90度向きを変えて取ってから手を外す
  (relative-move-hand 280 -550 870 pi pi/2 pi/2)
  (send *ri* :wait-interpolation nil)
  (relative-move-hand 600 0 870 (/ -pi 2) (/ pi 2) pi/2)
  ;;(send *fetch* :rarm :angle-vector #f(-38.915 26.4334 -161.297 63.6166 51.1904 110.028 141.666))
  ;;(send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :use-torso nil)
  ;;(send *fetch* :rarm :angle-vector #f(-18.915 26.4334 -161.297 63.6166 51.1904 110.028 141.666))
  ;;(send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :use-torso nil))
  )
|#

#| 足元に置かれたUberEatsの蓋を開ける |#
(defun box-open-improve ()

  (look-at-coord (make-relative-coord -150 -500 580))
  ;;腰を上げてからのほうが手がリュックに当たらない
  (send *fetch* :torso_lift_joint :joint-angle 100)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation nil)

  (relative-move-hand 100 -500 600 pi 0 pi/2) ;;予備動作
  (send *ri* :wait-interpolation nil)
  (relative-move-hand -300 -500 550 pi 0 pi/2 nil) ;;中に入れる
  (send *ri* :wait-interpolation nil)
  (unix::sleep 1) ;; sleepさせないと挙動がおかしい

  ;;(setq shoulder-y-angle-10 (- (send *fetch* :rarm-shoulder-y :joint-angle) 10))
  ;;(send *fetch* :rarm-shoulder-y :joint-angle shoulder-y-angle-10)
  ;;(send *fetch* :rarm :angle-vector #f(-60.9601 55.1198 -152.558 64.3187 71.9964 114.228 173.586)) ;; 元のものから１つめを-20度した
  
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :use-torso nil)
  (send *ri* :wait-interpolation nil)
  ;; ここから蓋開け動作
  (relative-move-hand -300 -500 650 pi 0 pi/2)
  (send *ri* :wait-interpolation nil)
  (relative-move-hand -250 -500 700 pi -0.15 pi/2)
  (send *ri* :wait-interpolation nil)
  (relative-move-hand -200 -500 800 pi -0.30 pi/2)
  (send *ri* :wait-interpolation nil)
  (relative-move-hand 100 -500 850 pi -0.45 pi/2)
  (send *ri* :wait-interpolation nil)
  (relative-move-hand 150 -500 750 pi -0.60 pi/2)
  (send *ri* :wait-interpolation nil)
  #|
  (relative-move-hand 280 -550 870 pi 0 pi/2)
  (send *ri* :wait-interpolation nil)
  |#
  ;; 90度向きを変えて取ってから手を外す
  (relative-move-hand 150 -500 750 pi pi/2 pi/2)
  (send *ri* :wait-interpolation nil)

  ;;(relative-move-hand 600 0 870 (/ -pi 2) (/ pi 2) pi/2)
  ;;(send *fetch* :rarm :angle-vector #f(-38.915 26.4334 -161.297 63.6166 51.1904 110.028 141.666))
  ;;(send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :use-torso nil)
  ;;(send *fetch* :rarm :angle-vector #f(-18.915 26.4334 -161.297 63.6166 51.1904 110.028 141.666))
  ;;(send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :use-torso nil))
  )

#| 箱をfetchの右横に置く動作 |#
(defun put-box ()
  ;;(relative-move-hand 350 -500 800 0 pi/2 pi/2)
  
  ;;(relative-move-hand 350 -500 800 0 pi/2 (* 3 pi/2))

  ;;(relative-move-hand -200 -400 750 -pi 0 pi/2)
  (relative-move-hand -250 -500 800 0 pi/2 (* 3 pi/2))

  (relative-move-hand -250 -500 600 0 pi/2 (* 3 pi/2))

  (relative-move-hand -30 -500 550 0 pi/2 (* 3 pi/2))

  (unix::sleep 1)
  (send *ri* :stop-grasp)
  (unix::sleep 1)
  (relative-move-hand -30 -500 800 0 pi/2 (* 3 pi/2))
  
  )

(defun box-flow ()
  (give-flow)
  (unix::sleep 1)

  ;;(move-to-place "/eng2/7f/room73B2-door-front")
  ;;(send *ri* :go-wait)
  (move-to-place "/eng2/7f/room73A3-front")
  (put-box)
  (unix::sleep 1)
  (box-open-improve)
  )
