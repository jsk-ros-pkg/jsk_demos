(load "package://fetcheus/fetch-interface.l")
(load "models/foldable-desk-object.l")
;;unless=when not
(unless (boundp '*ri*) (fetch-init))
(objects (list *fetch*))
(setq flag 0)
(setq before_go_flag 0)
(setq go_flag 0)
(setq wear_flag 0)
(setq time_pre (ros::time 0))

(ros::roseus-add-msgs "jsk_recognition_msgs")

;; クラスタリング結果であるBoundingBoxのtopic名
(defvar *topic-name* "/head_camera_remote/depth_registered/boxes")
(defvar *bounding-box-list* nil)

;; ros::initする
(ros::roseus "boundingboxarray_subscriber")

(ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "h_limit_max" :int 20)
(ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "h_limit_min" :int -20)
(ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "s_limit_max" :int 255)
(ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "s_limit_min" :int 80)
(ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "i_limit_max" :int 255)
(ros::set-dynamic-reconfigure-param "/head_camera_remote/depth_registered/hsi_filter" "i_limit_min" :int 50)

;;帽子をつかむための関数
(defun set-wear-func ()
  (defun before-catch ()
    (send *fetch* :angle-vector
	  #f(59.7386 85.6014 -6.15112 -99.6093 48.2162 7.19931 74.9269 75.0364 -0.90638 17.501)))
  (defun raise-hand ()
    (send *fetch* :angle-vector
	  #f(105.34 80.0424 -43.3069 0.366239 -52.9459 -12.1806 -56.887 1.18627 0.181273 18.8633)))
  ;;add---------------------------------------------------
  (defun before-wear ()
    (send *fetch* :angle-vector
	  #f(105.34 90.0000 -43.3069 0.366239 -52.9459 -12.1806 -56.887 181.18627 0.181273 18.8633)))
  ;;changed-----------------------------------------------
  (defun wear-hat ()
    (send *fetch* :angle-vector
	  #f(105.286 90.0 -42.5159 1.09134 -57.3843 -8.95059 -81.3646 -178.858 -0.05665 18.8194)))
  ;;------------------------------------------------------
  (defun catch-pose ()
    (send *fetch* :angle-vector
	  #f(102.021 58.0258 3.4729 77.8199 -59.5596 10.3194 -78.2664 74.1575 0.305897 15.3257)))
  (defun reset-pose ()
    (send *ri* :angle-vector (send *fetch* :reset-pose) 10000))
  (defun grasp-finger ()
    ;;effortが足りていればstart-graspでよい。
    ;;(send *ri* :start-grasp :wait t))
    (send *ri* :go-grasp :effort 70 :pos 0 :wait t))
  (defun release-finger ()
    ;;(send *ri* :stop-grasp :wait t))
    (send *ri* :go-grasp :effort 70 :pos 10 :wait t))
  (defun after-wear ()
    (send *fetch* :angle-vector
	  #f(105.34 80.0424 -43.3069 0.366239 -52.9459 -12.1806 -56.887 -178.858 0.181273 18.8633)))
)


;;近づく前にやること （手をあげておく）
(defun before_go ()
  (set-wear-func)
  (release-finger)
  (reset-pose)
  (send *ri* :wait-interpolation)
  ;;changed-----------------------------------
  (send *ri* :angle-vector (raise-hand) 10000)
  ;;-----------------------------------------
  (send *ri* :wait-interpolation)
  (setq before_go_flag 1))


;;赤色を見つけて近づく
(defun go_to_red ()
  (ros::sleep 3)
  ;;(send *ri* :speak-jp "帽子を発見しました。") ;;確認のため
  ;;(format t "bounding box ~A~%" (send *target-coords* :worldpos))
  ;; 物体を見る。こうしないと近づきすぎた時見えなくなる。
  (send *fetch* :head :look-at (send *target-coords* :worldpos)) 
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  
  (format t "b is [~A]~%" (send *target-coords* :worldcoords))
  ;;elt vector index->vector[index]
  (setq x (elt (send *target-coords* :worldpos) 0))
  (setq y (elt (send *target-coords* :worldpos) 1))
  (setq z (elt (send *target-coords* :worldpos) 2))
  (if (and (> x 910))
      (cond ((> y 200) ;;物体が自分より右にあれば
	     (send *ri* :go-velocity 0 0 (atan (/ y x)))
	     (ros::ros-info "turn-left"))
	    ((< y -200) ;;物体が自分より左にあれば
	     (send *ri* :go-velocity 0 0 (atan (/ y x)))
	     (ros::ros-info "turn-right"))
	    (t (send *ri* :clear-costmap)
	       (send *ri* :go-velocity (* (- x 900) 0.0008) 0 0) ;;物体が遠ければ
	       (ros::ros-info "go")))
      (progn
	(send *ri* :speak-jp "移動終了")
	(send *ri* :go-velocity 0 0 0)
	(setq go_flag 1))
      )
  (setq time_pre (ros::time-now)))


;;帽子をかぶる
(defun fetch_wear ()
  (set-table)
  (set-wear-func)
  ;;(send *ri* :speak-jp "帽子をつかみます。")
  ;;目標点を少し手前にずらしてikを解く
  ;;changed-----------------------------------------------
  (send *fetch* :inverse-kinematics (make-coords :pos (v+ (send *target-coords* :worldpos) #f(-0.1 0 0.15))))
  ;;----------------------------------------------------- 
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  ;;(send *ri* :speak-jp "逆運動学をときました。")
  ;;changed-----------------------------------------
  ;;前進して確実に帽子をつかむ
  ;;(send *ri* :speak-jp "前進します。")
  (send *ri* :go-velocity 0.05 0 0)
  (send *ri* :wait-interpolation)
  ;;-----------------------------------------------
  (grasp-finger)
  ;;(send *ri* :speak-jp "帽子をつかみました。")
  (send *ri* :wait-interpolation)
  ;;changed------------------------------------
  (send *ri* :angle-vector (before-wear) 10000)
  ;;------------------------------------------
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (wear-hat) 6000)
  (send *ri* :wait-interpolation)
  (release-finger)
  (send *ri* :speak-jp "サンタになったよ") 
  (send *ri* :wait-interpolation)
  (setq wear_flag 1)
  )


;;机から離れて手を戻す。
(defun after_wear ()
  ;;(send *ri* :speak-jp "机から離れます。")
  (set-wear-func)
  (send *ri* :go-pos-unsafe 0 0 -180)
  (send *ri* :wait-interpolation)
  (send *ri* :go-velocity 0.3 0 0)
  (send *ri* :angle-vector (after-wear) 7000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (catch-pose) 10000)
  (send *ri* :wait-interpolation)
  (reset-pose)
  (send *ri* :wait-interpolation)
  )

;;机にぶつからないよう干渉計算
(defun set-table ()
  (setq *table* (instance foldable-desk-object :init))
  (send *table* :translate #f(800 0 0))
  (setq *co* (instance collision-object-publisher :init))
  (send *co* :add-object *table* :frame-id "base_link" :relative-pose (make-coords :pos #f(800 0 0) ))
)


(defun bounding-box-array-cb (msg)
  (format t "bounding-box-array-cb~%")
  (setq *bounding-box-list* (send msg :boxes)) ;; boxesは、BoundingBoxのArray(Euslispではlist)
 
  ;; BoundingBoxがあれば表示する
  (when *bounding-box-list*
    (send *irtviewer* :draw-objects :flush nil) 
    (setq *target-coords* nil)
    (mapcar #'(lambda (b)
                ;; BoundingBoxは、dimsntion(直方体の幅・奥行き・高さ)をもつ
                (let* ((dims (ros::tf-point->pos (send b :dimensions)))
                       (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2)))
                       ;; (1) BoundingBoxのカメラ相対の座標系は、geometry_msgs/Poseという型で得られるので、Euslispのcoordsに変換する
                       (cam->obj-coords (ros::tf-pose->coords (send b :pose)))
                       ;; (2) *dxl-armed-turtlebot*モデルがカメラの座標系をもってるので、base_link基準の座標を取得する
                       (base-coords (send (send *fetch* :base_link_lk) :copy-worldcoords)))
                  ;; (3) Euslisp内部でのworld座標系の値にして、そこにmake-cubeの箱を設置する
                  (send bx :newcoords (send base-coords :transform cam->obj-coords))
                  (send bx :worldcoords)
		  ;;(format t "bounding box ~A ~A~%" dims (send bx :worldcoords))
		 
		  (if (and (< (elt (send (send bx :copy-worldcoords) :worldpos) 0) 2000)
			   (< -1000 (elt (send (send bx :copy-worldcoords) :worldpos) 1) 1000)
			   (> (elt dims 0) 100))
		      (progn
			(format t "bounding santa box ~A ~A~%" dims (send bx :worldcoords))
			(send bx :draw-on :flush nil :color #f(1 0 0)) ;; 描画
			;;追加 *target-coords*をグローバル変数として取ってくる。make-coordsのかたちになっている
			(setq *target-coords* (send bx :copy-worldcoords)))
		      (progn
			(send bx :draw-on :flush nil :color #f(0 1 0)) ;; 描画
			))

                  bx))
            *bounding-box-list*)
    (send *irtviewer* :viewer :viewsurface :flush)

    ;;帽子に近づいてつかんでかぶる。
    (if (and (= before_go_flag 0) (not (null *target-coords*)))
	(before_go))
    (if (and (= before_go_flag 1)
	     (= go_flag 0)
	     (not (null *target-coords*))
	     (ros::time> (send msg :header :stamp) time_pre))
	(progn
	  (go_to_red)))
    (if (and (= go_flag 1) (not (null *target-coords*)))
	(fetch_wear))
    ))

(ros::subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray #'bounding-box-array-cb 1)

(while (= wear_flag 0)
  (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
  (ros::spin-once)
  (ros::sleep)
  )

(ros::unsubscribe *topic-name*)

;;干渉解除
(send *co* :wipe-all)
;;帽子をかぶった後の動作。
(after_wear)




