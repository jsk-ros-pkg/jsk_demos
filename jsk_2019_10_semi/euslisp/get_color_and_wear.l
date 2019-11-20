(load "package://fetcheus/fetch-interface.l")
;;unless=when not
(unless (boundp '*ri*) (fetch-init))
(objects (list *fetch*))
(setq flag 0)
(setq before_go_flag 0)
(setq go_flag 0)

(ros::roseus-add-msgs "jsk_recognition_msgs")

;; クラスタリング結果であるBoundingBoxのtopic名
;;(defvar *topic-name* "/camera/depth_registered/cluster_decomposer/boxes")
(defvar *topic-name* "/head_camera/depth_registered/boxes")
(defvar *bounding-box-list* nil)

;; ros::initする
(ros::roseus "boundingboxarray_subscriber")

;;帽子をつかむための関数
(defun before-catch ()
  (send *fetch* :angle-vector #f(102.021 58.0258 3.4729 77.8199 -59.5596 10.3194 -78.2664 74.1575 0.305897 15.3257)))
  
(defun catch-hat ()
  (send *fetch* :angle-vector #f(102.021 52.4447 3.62671 77.8638 -59.5816 10.2755 -78.2444 74.0916 0.362546 16.8858)))	

(defun raise-hand ()
  (send *fetch* :angle-vector #f(9.65126 88.5018 -44.9988 3.72806 -58.6148 3.74963 -33.0027 -4.39479 0.283234 14.4029)))
 
(defun wear-hat ()
  (send *fetch* :angle-vector #f(12.4436 88.5897 -41.2854 2.98099 -58.6807 -0.908599 -83.1883 -4.35084 0.362546 13.9854)))

(defun after-wear ()
  (send *fetch* :angle-vector #f(12.4436 88.5238 -41.2854 2.95901 -58.6368 -0.820709 -28.2786 -4.35084 0.203922 13.9854)))

(defun reset-pose ()
  (send *ri* :angle-vector (send *fetch* :reset-pose) 10000))

(defun grasp-finger ()
  ;;effortが足りていればstart-graspでよい。
  ;;(send *ri* :start-grasp :wait t))
  (send *ri* :go-grasp :effort 70 :pos 0 :wait t))

(defun release-finger ()
  ;;(send *ri* :stop-grasp :wait t))
  (send *ri* :go-grasp :effort 70 :pos 10 :wait t))


;;近づく前にやること
(defun before_go ()
  (release-finger)
  (reset-pose)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (before-catch) 7000)
  (send *ri* :wait-interpolation)
  (setq before_go_flag 1))

;;赤色を見つけて近づく
(defun go_to_red_old (b)
  (send *ri* :speak-jp "帽子を発見しました。") ;;確認のため
  (while (= go_flag 0)
    (progn
      (format t "b is [~A]~%" (send b :worldcoords))
      (if (> (send (send (send b :pose) :position) :z) 0.6)	
	  (cond ((> (send (send (send b :pose) :position) :x) 0.1)
		 (send *ri* :go-velocity 0 0 (deg2rad -10)))
		((< (send (send (send b :pose) :position)  :x) -0.1)
		 (send *ri* :go-velocity 0 0 (deg2rad 10)))
		(t (send *ri* :go-velocity 0.1 0 0)))
	  (ros::ros-info "go-velocity is [~A]" (send *ri* :go-velocity))
	  (progn
	    (setq go_flag 1)
	    (send *ri* :speak-jp "移動終了")
	    (send *ri* :go-velocity 0 0 0)
	    )))
    ))

(defun go_to_red ()
  (send *ri* :speak-jp "帽子を発見しました。") ;;確認のため
  (format t "b is [~A]~%" (send *target-coords* :worldcoords))
  (setq x (elt (send *target-coords* :worldpos) 0))
  (setq y (elt (send *target-coords* :worldpos) 1))
  (setq z (elt (send *target-coords* :worldpos) 2))
  (if (> x 600) ;;elt vector index->vector[index]	
      (cond ((> y 200)
	     (send *ri* :go-velocity 0 0 (atan (/ y x)))
	     (ros::ros-info "turn-left"))
	    ((< y -200)
	     (send *ri* :go-velocity 0 0 (atan (/ y x)))
	     (ros::ros-info "turn-right"))
	    (t (send *ri* :go-velocity 0.1 0 0)
	       (ros::ros-info "go")))
      (progn
	(send *ri* :speak-jp "移動終了")
	(send *ri* :go-velocity 0 0 0)
	(setq go_flag 1))
      ))


;;ここに帽子をかぶる動作を書く
(defun fetch_wear ()
  (send *ri* :angle-vector (catch-hat) 6000)
  (send *ri* :wait-interpolation)
  (unix:sleep 3)
  (grasp-finger)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (raise-hand) 10000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (wear-hat) 6000)
  (send *ri* :wait-interpolation)
  (release-finger)
  (send *ri* :wait-interpolation) 
  )

;;机から離れて手を戻す。
(defun after_wear ()
  (send *ri* :go-velocity -0.5 0 0)
  (send *ri* :angle-vector (after-wear) 7000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (catch-hat) 10000)
  (send *ri* :wait-interpolation)
  (reset-pose)
  (send *ri* :wait-interpolation)
  (setq flag 1))
  

(defun bounding-box-array-cb (msg)
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
                       (cam-coords (send (send *fetch* :head_camera_rgb_optical_frame_lk) :copy-worldcoords)))		       ;;(base_link_coords (send (send *fetch* :base_link_lk))))
                  ;; (3) Euslisp内部でのworld座標系の値にして、そこにmake-cubeの箱を設置する
                  (send bx :newcoords (send cam-coords :transform cam->obj-coords))
                  (send bx :worldcoords)
                  (send bx :draw-on :flush nil :color #f(1 0 0)) ;; 描画

		  ;;追加 *target-coords*をグローバル変数として取ってくる。make-coordsのかたちになっている。
		  (setq *target-coords* (send bx :copy-worldcoords))

                  bx))
            *bounding-box-list*)
    
    (when (= flag 0)
      ;;ここで帽子をかぶる
      (cond ((= before_go_flag 0)
	     (before_go))
	    ((= go_flag 0)
	     (go_to_red))
	    (t (progn
		 (fetch_wear)
		 (after_wear)))
	    ))
	
    (send *irtviewer* :viewer :viewsurface :flush)
    ))

(ros::subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray #'bounding-box-array-cb 1)

(do-until-key
 (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
 (ros::spin-once)
 (ros::sleep)
 )
