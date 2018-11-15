;; robotの初期化
(load "init.l")
(init)
;; jsk_pcl_rosのmsgを使うため、loadする
;;(ros::load-ros-manifest "jsk_pcl_ros")
(ros::load-ros-manifest "jsk_recognition_msgs")

;; クラスタリング結果であるBoundingBoxのtopic名
;;(defvar *topic-name* "/camera/depth_registered/cluster_decomposer/boxes")
(defvar *topic-name* "/HSI_color_filter/boxes")
(defvar *bounding-box-list* nil)
(defvar *bx-list* nil)
;; ros::initする
(ros::roseus "boundingboxarray_subscriber")

;; コールバック関数
(defun bounding-box-array-cb (msg)
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
		  (print (length *bx-list*))
                  (send bx :draw-on :flush nil :color #f(1 0 0)) ;; 描画
		  bx))
            *bounding-box-list*)
    (send *irtviewer* :viewer :viewsurface :flush)
    )
  )

(ros::subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray #'bounding-box-array-cb 1)

(defun reach-arm nil
  (if (= (length tmp) 1)
      (progn
	(setq target-coords (make-coords :pos (send (car tmp) :worldpos)))
	(setq temae-coords (send (send target-coords :copy-worldcoords) :translate #f(-50 0 0)))
	(send *ri* :angle-vector #f(23.102 57.2229 -47.2485 -75.0006 87.4081 -147.508 20.6889 -154.0 -4.25995 13.6359) 5000)
	(send *ri* :wait-interpolation)
	(send *ri* :stop-grasp)
	(send *ri* :angle-vector (send *fetch* :inverse-kinematics temae-coords)  5000)
	(send *ri* :wait-interpolation)
	(send *ri* :angle-vector (send *fetch* :inverse-kinematics target-coords)  5000)
	(send *ri* :wait-interpolation)
	(send *ri* :start-grasp)
	(send *ri* :stop-grasp)
	(send *ri* :angle-vector #f(23.102 57.2229 -47.2485 -75.0006 87.4081 -147.508 20.6889 -154.0 -4.25995 13.6359) 5000)
	(send *ri* :wait-interpolation)
	(send *ri* :angle-vector (send *fetch* :reset-pose) 5000)
	(send *ri* :wait-interpolation)
	)
    (print "more than 1 objects were detected")
  ))

(do-until-key
 (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
 (when *bx-list*
   (setq tmp *bx-list*)
   (reach-arm)
   (exit)
   )
 (ros::spin-once)
 (ros::sleep)
 )
