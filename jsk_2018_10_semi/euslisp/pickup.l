(load "arm-movement-functions.l")
(load "models/foldable-desk-object.l")
;;(ros::load-ros-manifest "jsk_pcl_ros")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "fetch_driver_msgs")

;; クラスタリング結果であるBoundingBoxのtopic名
(defvar *bounding-box-topic* "/delivery_box_pickup/cluster_point_indices_decomposer_align_boxes_with_plane/boxes")
(defvar *box-address-topic* "/delivery_box_pickup/box_address")
(defvar *bounding-box-list* nil)
(defvar *bx-list* nil)
(defvar *length-threshold* 180.0)
(setq watching-pose #f(353.747 -84.1712 40.9718 100.495 92.6595 -130.04 -78.7594 -86.5444 -0.317235 50.5719))
;;(setq *co* (instance collision-object-publisher :init))
(setq *table* (instance foldable-desk-object :init))
;; ros::initする
(ros::roseus "pick-up-box")


;;*bounding-box-list*をsubscribeiして, 各boxの座標を計算する. 各座標は*bx-list*の中の, 各要素 :worldcoords になる?(
(send *fetch* :angle-vector (send *ri* :state :potentio-vector))
(defun detect-box-cb (msg)
  (setq a (send *ri* :state :angle-vector))
  (send *fetch* :angle-vector a)
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
                  ;; (3) Euslisp内部でのworld座標系の値にして、そこにmake-cubeの荷物を設置する
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

;;call back でsubscribeするbouding-box*に関する設定
(ros::subscribe *bounding-box-topic* jsk_recognition_msgs::BoundingBoxArray #'detect-box-cb 1)


(defun reach-arm (bx-list) 
  (let (box-coords)
    (print "picking up...")
    ;; collison object を作る
    ;;*fetch*を毎回reset-poseにして同じようにikを解かせる
    ;;(setq a (send (send (send (car *bx-list*) :copy-worldcoords) :translate #f(-100 0 -10)) :rotate (/ pi 5) :y))
    
    (send *fetch* :reset-pose)
    (send *fetch* :angle-vector #f(378.434 38.8984 17.8564 -88.2018 100.007 -251.993 90.7448 124.849 0.0 0.0))
    ;;ikを解きやすい場所に一回移動
    (setq box-coords (send (car bx-list) :copy-worldcoords))
    (send *ri* :stop-grasp)
    (send *ri* :angle-vector (send *fetch* :inverse-kinematics (send (send (send box-coords :copy-worldcoords) :translate #f(0.0 0.0 -150.0)) :rotate (/ pi -2) :y)) 10000);;一旦上に
    (print "upper pos...")
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector (send *fetch* :inverse-kinematics (send (send (send box-coords :copy-worldcoords) :translate #f(0.0 0.0 -35.0)) :rotate (/ pi -2) :y)) 10000);;つかむ場所まで下げる
    (print "downing...")
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp :effort 60 :wait t);;つかむ
    (print "graspping...")
    (send *ri* :angle-vector (send *fetch* :inverse-kinematics (send (send (send box-coords :copy-worldcoords) :translate #f(0.0 0.0 -150.0)) :rotate (/ pi -2) :y)) 2000);;もう一度上に持ち上げる
    (send *ri* :wait-interpolation)
    ) 
  )

(setq address "red")
;
;;宛先に応じたカゴに荷物を入れ、カゴに対応するrosparamをincrementする
(defun store-box nil
  (let (address)
    (progn
      (print "cheking address and storing box ...")
      (setq address (check-address))
      (store-box-to-basket address)
      (ros::set-param (format nil "/box_number/~A" address) (+ (ros::get-param (format nil "/box_number/~A" address)) 1))
      (format t "address of bos is ~A" address )
      (print "sotring box")
      )
    )
  )


;;荷物を全て拾い上げたことをpublishする
(defun publish-pickup-done nil
  (print "done")
  )

(defun set-collision-object nil
  (send *co* :wipe-all)
  (send *co* :add-object  *table* :frame-id "base_link" :relative-pose (make-coords :pos (float-vector 600 0 0))) ;;tableをcollision objectとして設定
  )

(defun initial-pose nil
  (send *ri* :angle-vector watching-pose 10000)
  (send *ri* :wait-interpolation)
  )

;;initialize
(setq pickup-status 1)

;; (setq a (send *ri* :state :angle-vector))
;; (send *fetch* :angle-vector a)

(defun max-length-edge (bbox)
  (car (sort (mapcar #'(lambda (edge) (send edge :length)) (send bbox :all-edges)) #'>=))
  )

(defun is-size-appropriate (bx-list)
  (if (< (car (sort (mapcar #'(lambda (bbox) (max-length-edge bbox)) bx-list) #'>=)) *length-threshold*) t
  nil)
  )

(do-until-key
 (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
 ;;箱たちの一変の長さの最大値が*length-threshold*を超えていなければ,つかむ動作に入る 
 (if (is-size-appropriate *bx-list*)
     (let (bx-list-tmp)
       (setq bx-list-tmp *bx-list*)
       (set-collision-object)
       (initial-pose)
       (reach-arm bx-list-tmp)
       (if (= pickup-status 1)
	   (store-box)
	 )
       (initial-pose)
       (unix:sleep 1)
       )
   (publish-pickup-done)
   )
 (send *co* :wipe-all)
 (ros::spin-once)
 (ros::sleep)
 )

